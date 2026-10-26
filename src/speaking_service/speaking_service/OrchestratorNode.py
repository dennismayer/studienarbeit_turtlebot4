import sys
from pathlib import Path
import os
from threading import Lock
import numpy as np

import rclpy
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.action.server import ActionServer, ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosidl_runtime_py import message_to_yaml
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from action_msgs.msg import GoalStatus
from speaking_interfaces.action import AudioCreation, AudioListener, AudioPlayer, AudioTranscriber, AudioOrchestrator
from speaking_interfaces.msg import ListenerParams, TranscriberParams, UploadParams

from speaking_service.scripts.cache import FileCache
from robot_common.parameter_helper import get_param

class SpeechOrchestratorNode(Node):

    def __init__(self):
        # defaults
        node_name = "AudioOrchestrator"

        super().__init__(node_name)

        # load parameters
        self.declare_parameters('', [
            ("action_name", "speechRequest",
             ParameterDescriptor(
                 description="Name of the action server that clients send speech requests to",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("creation_action", "/AudioWorkers/audioCreation",
             ParameterDescriptor(
                 description="Fully qualified name of the AudioCreationWorker action server",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("player_action", "/AudioWorkers/audioPlayer",
             ParameterDescriptor(
                 description="Fully qualified name of the AudioPlayerWorker action server",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("listener_action", "/AudioWorkers/audioListener",
             ParameterDescriptor(
                 description="Fully qualified name of the AudioListenerWorker action server",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("transcriber_action", "/AudioWorkers/audioTranscriber",
             ParameterDescriptor(
                 description="Fully qualified name of the AudioTranscriberWorker action server",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("action_timeout", 5.0,
             ParameterDescriptor(
                 description="Timeout in seconds for action calls",
                 read_only=True,
                 type=ParameterType.PARAMETER_DOUBLE)),
            ("audio_cache_path", "speaking_service/audio_cache",
             ParameterDescriptor(
                 description="Audio cache directory, relative to $ROS_HOME",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
        ])

        # variables
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self.cache_dir = ros_home / get_param(self, "audio_cache_path", str)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.file_cache = FileCache(self.cache_dir, self.get_logger())
        self.lock_callback = Lock()

        self.__action_timeout = get_param(self, "action_timeout", float)
        
        self.first_noise_detected = False
        self._current_goal_handle = None
        self._active_worker_handle = None
        
        
        self.get_logger().info("Starting Action Clients ...")
        
        self.__audio_creation_action_client = ActionClient(
            self, 
            AudioCreation,
            get_param(self, "creation_action", str)
        )
        
        self.__audio_player_action_client = ActionClient(
            self, 
            AudioPlayer,
            get_param(self, "player_action", str)
        )
        
        self.__audio_listener_action_client = ActionClient(
            self, 
            AudioListener,
            get_param(self, "listener_action", str)
        )

        self.__audio_transcriber_action_client = ActionClient(
            self, 
            AudioTranscriber,
            get_param(self, "transcriber_action", str)
        )
        
        # check if clients are up
        self.check_if_client_started(self.__audio_creation_action_client)
        self.check_if_client_started(self.__audio_player_action_client)
        self.check_if_client_started(self.__audio_listener_action_client)
        self.check_if_client_started(self.__audio_transcriber_action_client)
        
        self.get_logger().info("Action Clients started!")
        
        self.get_logger().info("Starting Action Server")
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioOrchestrator, 
            action_name=get_param(self, "action_name", str),
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            goal_callback=self.goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_logger().info("Started Server!")
        
    async def execute_callback(self, goal_handle: ServerGoalHandle) -> AudioOrchestrator.Result:
        # recording handed over by the listener; the orchestrator owns it and deletes it in finally
        recording_path: Path | None = None
        try:
            self.get_logger().info("Received goal...")
            self.get_logger().debug(f"Received goal:\n{message_to_yaml(goal_handle.request)}")
            
            request: AudioOrchestrator.Goal = goal_handle.request
            
            feedback = AudioOrchestrator.Feedback()
            
            self.get_logger().debug("Search cache for text file...")
            file_path_duration_object = self.file_cache.lookup(request.question_or_sentence)
            
            if not file_path_duration_object:
                self.get_logger().debug("Couldn't find file; generating...")
                feedback.state = AudioOrchestrator.Feedback.CREATING_AUDIO
                goal_handle.publish_feedback(feedback)
                if goal_handle.is_cancel_requested:
                    return self._handle_cancel_callback(goal_handle)
                
                goal_msg = AudioCreation.Goal()
                goal_msg.text = request.question_or_sentence
                
                response, error = await self._run_worker(
                    self.__audio_creation_action_client, goal_msg, timeout_sec=45.0
                )
                
                if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
                    if response is not None and response.status == GoalStatus.STATUS_CANCELED:
                        return self._handle_cancel_callback(goal_handle)
                    return self._handle_error_callback(goal_handle, f"Failed to synthesis: {error}")
                
                creation_result: AudioCreation.Result = response.result

                file_path_duration_object = {"file_path": creation_result.file_path, "duration": creation_result.duration}
                try:
                    self.file_cache.store(os.path.basename(file_path_duration_object["file_path"]), file_path_duration_object["duration"])
                except Exception as e:
                    return self._handle_error_callback(goal_handle, f"Failed to synthesis cache store: {e}")
                self.get_logger().info("Synthesising finished!")
            else:
                self.get_logger().info("File found in cache!")
            
            # Synthesising data
            self.get_logger().info("Start playing...")
            goal_msg = AudioPlayer.Goal()
            goal_msg.file_path = file_path_duration_object["file_path"]
            
            feedback.state = AudioOrchestrator.Feedback.PLAYING
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                return self._handle_cancel_callback(goal_handle)
            
            response, error = await self._run_worker(
                self.__audio_player_action_client, goal_msg, self._feedback_player_node, timeout_sec=45.0
            )
            
            if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
                if response is not None and response.status == GoalStatus.STATUS_CANCELED:
                    return self._handle_cancel_callback(goal_handle)
                return self._handle_error_callback(goal_handle, f"Failed to play audio: {error}")
            
            self.get_logger().info("Playback finished!")
            
            if request.response_handling == AudioOrchestrator.Goal.NO_RESPONSE:
                self.get_logger().info("Finished with NoResponse...")
                goal_handle.succeed()
                return self.assemble_result(True, "NoResponse", "")
            
            # Recording data
            self.get_logger().info("Start Recording...")
            goal_msg = AudioListener.Goal()
            goal_msg.max_duration = request.listener_parameters.max_duration
            goal_msg.silence_duration = request.listener_parameters.silence_duration
            goal_msg.start_timeout = request.listener_parameters.start_timeout
            
            feedback.state = AudioOrchestrator.Feedback.WAITING_FOR_SPEECH
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                return self._handle_cancel_callback(goal_handle)
            
            self._current_goal_handle = goal_handle
            response, error = await self._run_worker(
                self.__audio_listener_action_client, goal_msg, self._feedback_listener_node, timeout_sec=45.0
            )
            self._current_goal_handle = None
            self.first_noise_detected = False

            # the listener also returns a file on Canceled/TotalTimeout if speech was heard
            if response is not None and response.result.file_path:
                recording_path = Path(response.result.file_path)

            if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
                if response is not None and response.status == GoalStatus.STATUS_CANCELED:
                    return self._handle_cancel_callback(goal_handle)

                return self._handle_error_callback(goal_handle, f"Failed to record audio: {error}")

            listener_result: AudioListener.Result = response.result

            if recording_path is None or not recording_path.is_file() or listener_result.duration == 0.0:
                return self._handle_error_callback(goal_handle, f"Recording doesn't exists or has duration 0!")
            
            self.get_logger().info("Finished Recording!")
            
            # check if upload response or do something else
            if request.response_handling == AudioOrchestrator.Goal.UPLOAD_RESPONSE:
                self.get_logger().info("Uploading data...")
                feedback.state = AudioOrchestrator.Feedback.UPLOADING
                goal_handle.publish_feedback(feedback)
                if goal_handle.is_cancel_requested:
                    return self._handle_cancel_callback(goal_handle)
                # TODO Upload Node adding


                self.get_logger().info("Finished Uploading!")
                goal_handle.succeed()
                return self.assemble_result(True, "UploadResponse", "")
            
            # last case: Transcribing
            #TODO Implement audios for mean time
            self.get_logger().info("transcribing data ...")    
            goal_msg = AudioTranscriber.Goal()
            goal_msg.file_path = listener_result.file_path
            goal_msg.initial_prompt = request.transcriber_parameters.initial_prompt
            goal_msg.language = "de"
            
            feedback.state = AudioOrchestrator.Feedback.TRANSCRIBING
            goal_handle.publish_feedback(feedback)
            if goal_handle.is_cancel_requested:
                return self._handle_cancel_callback(goal_handle)
                          
            response, error = await self._run_worker(
                self.__audio_transcriber_action_client, goal_msg, self._feedback_transcriber_node, timeout_sec=45.0
            )
            
            if response is None or response.status != GoalStatus.STATUS_SUCCEEDED:
                if response is not None and response.status == GoalStatus.STATUS_CANCELED:
                    return self._handle_cancel_callback(goal_handle)
                return self._handle_error_callback(goal_handle, f"Failed to transcribe audio: {error}")
            
            transcriber_result: AudioTranscriber.Result = response.result
            
            if (np.isnan(transcriber_result.avg_logprob) or np.isnan(transcriber_result.no_speech_prob) or
                transcriber_result.avg_logprob < request.transcriber_parameters.min_avg_logprob or 
                transcriber_result.no_speech_prob > request.transcriber_parameters.max_no_speech_prob):
                self.get_logger().info("Transcription with less accuracy")
                self.get_logger().info(f"avg_logprob: min: {request.transcriber_parameters.min_avg_logprob}; actually: {transcriber_result.avg_logprob}")
                self.get_logger().info(f"no_speech_prob: max{request.transcriber_parameters.max_no_speech_prob}; actually: {transcriber_result.no_speech_prob}")
                #TODO Implement retry logic
            
            goal_handle.succeed()
            return self.assemble_result(True, "TranscribeResponse", transcriber_result.text)
        
        except Exception as e:
            return self._handle_error_callback(goal_handle, f"An Unknown Error occurred: {e}")
            
        finally:
            # covers every exit: success, error, cancel and exceptions
            # (after a successful transcription the transcriber already deleted it, so this is a no-op)
            if recording_path is not None:
                try:
                    recording_path.unlink(missing_ok=True)
                except OSError as e:
                    self.get_logger().warning(f"Could not delete recording {recording_path}: {e}")
            self.lock_callback.release()

    async def _run_worker(self, client: ActionClient, goal, feedback_callback=None, *, timeout_sec: float):
        if not client.server_is_ready():
            return None, "ServerNotAvailable"

        # goal acceptance should be quick, so the short action_timeout fits here
        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        worker_handle: ClientGoalHandle | None = await self._await_with_timeout(send_future, self.__action_timeout)
        if send_future.cancelled():
            return None, "GoalResponseTimeout"
        if worker_handle is None or not worker_handle.accepted:
            return None, "GoalRejected"

        self._active_worker_handle = worker_handle
        try:
            result_future = worker_handle.get_result_async()
            response = await self._await_with_timeout(result_future, timeout_sec)
            if result_future.cancelled():
                # not awaited: a dead worker would never answer the cancel request either
                worker_handle.cancel_goal_async()
                return None, "ResultTimeout"
        finally:
            self._active_worker_handle = None

        if response is None:
            return None, "NoResult"
        return response, ""

    def check_if_client_started(self, client: ActionClient):
        if client.wait_for_server(timeout_sec=45.0):
            return 
        
        raise RuntimeError(f"Action not reachable: {client._action_name}")
    
    def _feedback_player_node(self, msg):
        feedback: AudioPlayer.Feedback = msg.feedback
        self.get_logger().info(f"Progress: {feedback.progress:.2f}")
        
    def _feedback_listener_node(self, msg):
        feedback: AudioListener.Feedback = msg.feedback
        
        if not self.first_noise_detected and feedback.state == AudioListener.Feedback.RECORDING:
            feedback_org = AudioOrchestrator.Feedback()
            feedback_org.state = AudioOrchestrator.Feedback.RECORDING
            if self._current_goal_handle is not None:
                self._current_goal_handle.publish_feedback(feedback_org)
            self.first_noise_detected = True
            
    def _feedback_transcriber_node(self, msg):
        feedback: AudioTranscriber.Feedback = msg.feedback
        self.get_logger().info(f"Progress: {feedback.progress:.2f}")
            
    def goal_callback(self, goal_request):
        if not self.lock_callback.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, cancel_request):
        if self._active_worker_handle is not None:
            self._active_worker_handle.cancel_goal_async()
        return CancelResponse.ACCEPT
    
    def assemble_result(self, success: bool, message: str, transcribed_text: str):
        result = AudioOrchestrator.Result()
        result.success = success
        result.message = message
        result.transcribed_text = transcribed_text
        return result
    
    def _handle_error_callback(self, goal_handle, error_msg):
        self.get_logger().error(error_msg)
        goal_handle.abort()
        return self.assemble_result(False, error_msg, "")
    
    def _handle_cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Canceling...")
        goal_handle.canceled()
        return self.assemble_result(False, "Canceled", "")
    
    async def _await_with_timeout(self, future, timeout_sec: float):
        def on_timeout():
            timer.cancel()
            future.cancel()          # wakes up the await below with None

        timer = self.create_timer(timeout_sec, on_timeout)
        try:
            return await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

        

def main(args=None):
    rclpy.init(args=args)

    orchestrator_node = None
    executor = None
    exit_code = 0

    try:
        orchestrator_node = SpeechOrchestratorNode()
        executor = MultiThreadedExecutor()
        executor.add_node(orchestrator_node)

        executor.spin()
    except RuntimeError as e:
        print(e, file=sys.stderr)
        exit_code = -1
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...", file=sys.stderr)
    except Exception as e:
        print(f"Exception occurred in main thread: {e}", file=sys.stderr)
        exit_code = -1
    finally:
        if executor is not None:
            executor.shutdown()
        if orchestrator_node is not None:
            orchestrator_node.destroy_node()
        try_shutdown()

    return exit_code

if __name__ == "__main__":
    sys.exit(main())