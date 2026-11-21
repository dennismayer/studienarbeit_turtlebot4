import sys
import traceback
from pathlib import Path
import os
from threading import Lock
import numpy as np
import random

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
from server_interfaces.action import UploadFile

from speaking_service.scripts.cache import FileCache
from robot_common.parameter_helper import get_param
from robot_common.action_client_server_helper import check_if_client_started, send_universal_goal, wait_for_universal_result, GoalAbort, GoalCanceled, GoalRejected
from robot_common.logger_helper import log_goal

DEFAULT_NODE_NAME = "AudioOrchestrator"
RESPONSE_HANDLING_NAMES = {
    AudioOrchestrator.Goal.NO_RESPONSE: "no response",
    AudioOrchestrator.Goal.UPLOAD_RESPONSE: "upload",
    AudioOrchestrator.Goal.TRANSCRIBE_RESPONSE: "transcribe",
}

class SpeechOrchestratorNode(Node):

    def __init__(self):
        super().__init__(DEFAULT_NODE_NAME)

        # Startup procedure
        self._declare_node_parameters()
        self._declare_node_variables()
        self._start_action_clients()
        self._start_action_servers()
         
    def _declare_node_parameters(self) -> None:
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
            ("uploader_action", "upload_file",
                ParameterDescriptor(
                    description="Fully qualified name of the UploadFile action server",
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("action_timeout", 5.0,
                ParameterDescriptor(
                    description="Timeout in seconds for action calls",
                    read_only=True,
                    type=ParameterType.PARAMETER_DOUBLE)),
            ("client_start_timeout", 30.0,
                ParameterDescriptor(
                    description="Timeout in seconds for each worker action server to become available at startup",
                    read_only=True,
                    type=ParameterType.PARAMETER_DOUBLE)),
            ("synthesis_timeout", 45.0,
                ParameterDescriptor(
                    description="Timeout in seconds for the AudioCreationWorker result",
                    read_only=True,
                    type=ParameterType.PARAMETER_DOUBLE)),
            ("player_timeout", 45.0,
                ParameterDescriptor(
                    description="Timeout in seconds for the AudioPlayerWorker result",
                    read_only=True,
                    type=ParameterType.PARAMETER_DOUBLE)),
            ("listener_timeout", 60.0,
                ParameterDescriptor(
                    description="Timeout in seconds for the AudioListenerWorker result",
                    read_only=True,
                    type=ParameterType.PARAMETER_DOUBLE)),
            ("transcriber_timeout", 45.0,
                ParameterDescriptor(
                    description="Timeout in seconds for the AudioTranscriberWorker result",
                    read_only=True,
                    type=ParameterType.PARAMETER_DOUBLE)),
            ("uploader_timeout", 60.0,
                ParameterDescriptor(
                    description="Timeout in seconds for the UploadFile result",
                    read_only=True,
                    type=ParameterType.PARAMETER_DOUBLE)),
            ("audio_cache_path", "speaking_service/audio_cache",
                ParameterDescriptor(
                    description="Audio cache directory, relative to $ROS_HOME",
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
        ])
        
    def _declare_node_variables(self) -> None:
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self._cache_dir = ros_home / get_param(self, "audio_cache_path", str)
        self._cache_dir.mkdir(parents=True, exist_ok=True)
        self._waiting_dir = ros_home / "speaking_service" / "waiting_audios"
        self._waiting_dir.mkdir(parents=True, exist_ok=True)
        self._file_cache = FileCache(self._cache_dir, self.get_logger())
        self._lock_callback = Lock()
        self._lock_handle_set = Lock()

        self.__action_timeout = get_param(self, "action_timeout", float)
        self.__client_start_timeout = get_param(self, "client_start_timeout", float)
        self.__synthesis_timeout = get_param(self, "synthesis_timeout", float)
        self.__player_timeout = get_param(self, "player_timeout", float)
        self.__listener_timeout = get_param(self, "listener_timeout", float)
        self.__transcriber_timeout = get_param(self, "transcriber_timeout", float)
        self.__uploader_timeout = get_param(self, "uploader_timeout", float)
        
        self._first_noise_detected = False
        self._current_goal_handle = None
        self._active_worker_handle = []

        self.get_logger().debug(f"Audio cache: {self._cache_dir}, filler audios: {self._waiting_dir}")
        try:
            self._get_file_path_of_random_filler()
        except ValueError:
            self.get_logger().warning(f"No filler audios (.wav) in {self._waiting_dir}, transcriptions run without filler")

    def _start_action_clients(self) -> None:
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
        
        self.__server_upload_client = ActionClient(
            self,
            UploadFile,
            get_param(self, "uploader_action", str)
        )
        
        # check if clients are up
        check_if_client_started(self.__audio_creation_action_client, self.__client_start_timeout)
        check_if_client_started(self.__audio_player_action_client, self.__client_start_timeout)
        check_if_client_started(self.__audio_transcriber_action_client, self.__client_start_timeout)
        check_if_client_started(self.__audio_listener_action_client, self.__client_start_timeout)
        check_if_client_started(self.__server_upload_client, self.__client_start_timeout)
        
        self.get_logger().info("All worker action servers reachable")

    def _start_action_servers(self) -> None:
        action_name = get_param(self, "action_name", str)
        self.__action_server = ActionServer(
            node=self,
            action_type=AudioOrchestrator,
            action_name=action_name,
            execute_callback=self._execute_callback,
            cancel_callback=self._cancel_callback,
            goal_callback=self._goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_logger().info(f"Action server '{action_name}' ready, waiting for speech requests")
    
    async def _execute_callback(self, goal_handle: ServerGoalHandle) -> AudioOrchestrator.Result:
        # recording handed over by the listener; the orchestrator owns it and deletes it in finally
        recording_path: Path | None = None
        try:
            request: AudioOrchestrator.Goal = log_goal(self, goal_handle)
            response_name = RESPONSE_HANDLING_NAMES.get(request.response_handling, f"unknown ({request.response_handling})")
            self.get_logger().info(f"Speech request: '{request.question_or_sentence}' (response: {response_name})")

            # checks the cache if the file already exists
            file_path_duration_object = self._file_cache.lookup(request.question_or_sentence)

            # checks of the audio has to be sythesised and does so if needed
            if not file_path_duration_object:
                file_path_duration_object = await self._synthesis_audio(request.question_or_sentence, goal_handle)
            else:
                self.get_logger().info("Audio found in cache, skipping synthesis")
            
            # Playing audio
            await self._play_audio(goal_handle, file_path_duration_object)
            
            # finish if no recording is needed
            if request.response_handling == AudioOrchestrator.Goal.NO_RESPONSE:
                return self._goal_no_response_handling(goal_handle)
            
            # Recording data only if it is nessessary
            if request.response_handling == AudioOrchestrator.Goal.UPLOAD_RESPONSE and not request.upload_parameters.send_file:
                listener_result = AudioListener.Result()
            else:
                listener_result: AudioListener.Result = await self._record_audio(goal_handle, request)
                recording_path = Path(listener_result.file_path)
            
            # check if upload response or do something else
            if request.response_handling == AudioOrchestrator.Goal.UPLOAD_RESPONSE:
                return await self._goal_upload_handling(goal_handle, listener_result.file_path, listener_result.duration)
            
            # last case: Transcribing
            return await self._goal_transcribing_handling(goal_handle, listener_result.file_path, request)
        
        except GoalCanceled:
            return self._handle_cancel_callback(goal_handle)
        
        except GoalAbort as e:
            return self._handle_error_callback(goal_handle, f"{e}")
        
        except GoalRejected as e:
            return self._handle_error_callback(goal_handle, f"{e}")

        except Exception as e:
            self.get_logger().error(traceback.format_exc())
            return self._handle_error_callback(goal_handle, f"An Unknown Error occurred: {e}")

        finally:
            # covers every exit: success, error, cancel and exceptions
            # (after a successful transcription the transcriber already deleted it, so this is a no-op)
            if recording_path is not None:
                try:
                    recording_path.unlink(missing_ok=True)
                except OSError as e:
                    self.get_logger().warning(f"Could not delete recording {recording_path}: {e}")

            # resets variables
            self._first_noise_detected = False
            self._current_goal_handle = None

            # cleans all processes
            with self._lock_handle_set:
                if self._active_worker_handle:
                    self.get_logger().debug(f"Canceling {len(self._active_worker_handle)} leftover worker goal(s)")
                for worker_handle in self._active_worker_handle:
                    worker_handle.cancel_goal_async()
                self._active_worker_handle.clear()
            
            self._lock_callback.release()

    def _send_state_feedback(self, state, goal_handle: ServerGoalHandle):
        feedback = AudioOrchestrator.Feedback()
        feedback.state = state
        goal_handle.publish_feedback(feedback)
        
    def _check_for_cancel_request(self, goal_handle: ServerGoalHandle):
        if goal_handle.is_cancel_requested:
            raise GoalCanceled()
    
    async def _synthesis_audio(self, question_or_sentence: str, goal_handle: ServerGoalHandle):
        self.get_logger().info("Audio not in cache, synthesising...")
        self._send_state_feedback(AudioOrchestrator.Feedback.CREATING_AUDIO, goal_handle)

        self._check_for_cancel_request(goal_handle)
        
        goal_msg = AudioCreation.Goal()
        goal_msg.text = question_or_sentence
        
        creation_result: AudioCreation.Result = await self._run_worker(
            self.__audio_creation_action_client, goal_msg, timeout_sec=self.__synthesis_timeout
        )
        
        file_path_duration_object = {"file_path": creation_result.file_path, "duration": creation_result.duration}
        try:
            self._file_cache.store(os.path.basename(file_path_duration_object["file_path"]), file_path_duration_object["duration"])
        except Exception as e:
            raise GoalAbort(f"Failed to synthesis cache store: {e}")
        self.get_logger().info(f"Synthesis finished ({creation_result.duration:.1f} s of audio)")
        
        return file_path_duration_object
    
    async def _play_audio(self, goal_handle: ServerGoalHandle, file_path_duration_object) -> None:
        self.get_logger().info(f"Playing audio ({file_path_duration_object['duration']:.1f} s)...")
        goal_msg = AudioPlayer.Goal()
        goal_msg.file_path = file_path_duration_object["file_path"]
        
        self._send_state_feedback(AudioOrchestrator.Feedback.PLAYING, goal_handle)
        self._check_for_cancel_request(goal_handle)
        
        await self._run_worker(
            self.__audio_player_action_client, goal_msg, feedback_callback=self._feedback_player_node, timeout_sec=self.__player_timeout
        )
        
        self.get_logger().info("Playback finished")
    
    async def _record_audio(self, goal_handle: ServerGoalHandle, request: AudioOrchestrator.Goal):
        goal_msg = AudioListener.Goal()
        goal_msg.max_duration = request.listener_parameters.max_duration
        goal_msg.silence_duration = request.listener_parameters.silence_duration
        goal_msg.start_timeout = request.listener_parameters.start_timeout
        self.get_logger().info(
            f"Listening for an answer (start timeout {goal_msg.start_timeout:.1f} s, "
            f"silence {goal_msg.silence_duration:.1f} s, max {goal_msg.max_duration:.1f} s)..."
        )

        self._send_state_feedback(AudioOrchestrator.Feedback.WAITING_FOR_SPEECH, goal_handle)
        self._check_for_cancel_request(goal_handle)
        
        self._current_goal_handle = goal_handle
        result: AudioListener.Result = await self._run_worker(
            self.__audio_listener_action_client, goal_msg, self._feedback_listener_node, timeout_sec=self.__listener_timeout
        )

        # the listener also returns a file on Canceled/TotalTimeout if speech was heard
        if result.file_path:
            recording_path = Path(result.file_path)
        else: 
            recording_path = None

        if recording_path is None or not recording_path.is_file():
            raise GoalAbort(f"Listener returned no recording file (path: '{result.file_path}')")

        if result.duration == 0.0:
            Path(recording_path).unlink(missing_ok=True)
            raise GoalAbort("Listener returned an empty recording (duration 0 s)")

        self.get_logger().info(f"Recording finished ({result.duration:.1f} s)")
        return result
    
    async def _send_goal(self, client: ActionClient, goal, feedback_callback=None) -> ClientGoalHandle:
        worker_handle = await send_universal_goal(self, client, goal, self.__action_timeout, feedback_callback=feedback_callback)

        with self._lock_handle_set:
            self._active_worker_handle.append(worker_handle)
        return worker_handle
    
    async def _wait_for_result(self, worker_handle: ClientGoalHandle, timeout_sec: float):
        try:
            response = await wait_for_universal_result(self, worker_handle, timeout_sec)
        finally:
            with self._lock_handle_set:
                self._active_worker_handle.remove(worker_handle)

        action_name = worker_handle._action_client._action_name
        
        match response.status:
            case GoalStatus.STATUS_SUCCEEDED:
                return response.result
            case GoalStatus.STATUS_CANCELED:
                self.get_logger().debug(f"Worker {action_name} confirmed the cancel")
                self._delete_result_file(response)
                raise GoalCanceled()
            case GoalStatus.STATUS_ABORTED:
                self._delete_result_file(response)
                raise GoalAbort(f"Worker {action_name} aborted: {response.result.message}")
            case _:
                self._delete_result_file(response)
                raise GoalAbort(f"Worker {action_name} finished with unexpected status {response.status}")

    def _delete_result_file(self, response):
        if hasattr(response.result, "file_path") and os.path.isfile(Path(response.result.file_path)):
            Path(response.result.file_path).unlink(missing_ok=True)
            self.get_logger().debug(f"Deleted leftover worker file {response.result.file_path}")
    
    async def _run_worker(self, client: ActionClient, goal, feedback_callback=None, *, timeout_sec: float):
        worker_handle = await self._send_goal(client, goal, feedback_callback)
        return await self._wait_for_result(worker_handle, timeout_sec=timeout_sec)
    
    def _goal_no_response_handling(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Request finished (no response requested)")
        goal_handle.succeed()
        return self._assemble_result(True, "NoResponse", "")
    
    async def _goal_upload_handling(self, goal_handle: ServerGoalHandle, file_path: str, duration: float):
        self._send_state_feedback(AudioOrchestrator.Feedback.UPLOADING, goal_handle)
        self._check_for_cancel_request(goal_handle)
        
        request: AudioOrchestrator.Goal = goal_handle.request
        
        goal = UploadFile.Goal()
        goal.file_path = file_path
        goal.user_id = request.upload_parameters.user_id
        goal.message = request.upload_parameters.message
        goal.sender_name = request.upload_parameters.sender_name
        goal.send_file = request.upload_parameters.send_file
        goal.audio_duration_s = round(duration)
        result: UploadFile.Result = await self._run_worker(
            self.__server_upload_client,
            goal,
            feedback_callback=self._feedback_uploader,
            timeout_sec=self.__uploader_timeout
        )
        if not result.success:
            raise GoalAbort(f"Upload failed with message: {result.message}")
        
        self.get_logger().info("Upload successful!")
        
        goal_handle.succeed()
        return self._assemble_result(True, "UploadResponse", "")
    
    def _feedback_uploader(self, msg):
        self.get_logger().info(f"State of Upload: {msg.feedback.state}")
    
    async def _goal_transcribing_handling(self, goal_handle: ServerGoalHandle, file_path: str, request):
        self.get_logger().info("Transcribing recording...")
        goal_msg = AudioTranscriber.Goal()
        goal_msg.file_path = file_path
        goal_msg.initial_prompt = request.transcriber_parameters.initial_prompt
        goal_msg.language = "de"
        
        self._send_state_feedback(AudioOrchestrator.Feedback.TRANSCRIBING, goal_handle)
        self._check_for_cancel_request(goal_handle)
        
        audio_goal_handle = await self._get_filler_goal_handle()
        transcriber_result: AudioTranscriber.Result = await self._run_worker(
            self.__audio_transcriber_action_client, goal_msg, self._feedback_transcriber_node, timeout_sec=self.__transcriber_timeout
        )
        await self._wait_for_filler(audio_goal_handle)
        
        self.get_logger().info(
            f"Transcription: '{transcriber_result.text}' "
            f"(avg_logprob {transcriber_result.avg_logprob:.2f}, no_speech_prob {transcriber_result.no_speech_prob:.2f})"
        )
        
        goal_handle.succeed()
        return self._assemble_result(True, "TranscribeResponse", transcriber_result.text)
    
    async def _get_filler_goal_handle(self) -> ClientGoalHandle | None:
        audio_goal_msg = AudioPlayer.Goal()
        
        # failed audio play should not cause transcribtion fail
        try:
            audio_goal_msg.file_path = self._get_file_path_of_random_filler()
            audio_goal_handle = await self._send_goal(self.__audio_player_action_client, audio_goal_msg)
            self.get_logger().debug(f"Playing filler {os.path.basename(audio_goal_msg.file_path)}")
        except Exception as e:
            self.get_logger().warning(f"No filler played: {str(e) or type(e).__name__}")
            audio_goal_handle = None
        return audio_goal_handle

    async def _wait_for_filler(self, goal_handle: ClientGoalHandle | None):
        # if something goes wrong just ignore it
        if goal_handle is not None:
            try:
                await self._wait_for_result(goal_handle, self.__player_timeout)
            except Exception as e:
                self.get_logger().warning(f"Filler playback did not finish: {str(e) or type(e).__name__}")
    
    def _get_file_path_of_random_filler(self):
        list_of_all_files = os.listdir(self._waiting_dir)
        list_of_wav_files = []
        
        for file in list_of_all_files:
            if file.endswith(".wav"):
                list_of_wav_files.append(file)
        
        if len(list_of_wav_files) == 0:
            raise ValueError("No files in directory")
        
        return str(self._waiting_dir / random.choice(list_of_wav_files))
    
    def _feedback_player_node(self, msg):
        feedback: AudioPlayer.Feedback = msg.feedback
        self.get_logger().debug(f"Playback progress: {feedback.progress:.2f}")

    def _feedback_listener_node(self, msg):
        feedback: AudioListener.Feedback = msg.feedback

        if not self._first_noise_detected and feedback.state == AudioListener.Feedback.RECORDING:
            self.get_logger().info("Speech detected, recording...")
            feedback_org = AudioOrchestrator.Feedback()
            feedback_org.state = AudioOrchestrator.Feedback.RECORDING
            if self._current_goal_handle is not None:
                self._current_goal_handle.publish_feedback(feedback_org)
            self._first_noise_detected = True
            
    def _feedback_transcriber_node(self, msg):
        feedback: AudioTranscriber.Feedback = msg.feedback
        self.get_logger().debug(f"Transcription progress: {feedback.progress:.2f}")

    def _goal_callback(self, goal_request):
        if not self._lock_callback.acquire(blocking=False):
            self.get_logger().warning("Rejected speech request: another request is still running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, cancel_request):
        self.get_logger().info("Cancel requested by client")
        with self._lock_handle_set:
            if self._active_worker_handle is not None:
                for worker_handle in self._active_worker_handle:
                    worker_handle.cancel_goal_async()
        return CancelResponse.ACCEPT
    
    def _assemble_result(self, success: bool, message: str, transcribed_text: str):
        result = AudioOrchestrator.Result()
        result.success = success
        result.message = message
        result.transcribed_text = transcribed_text
        return result
    
    def _handle_error_callback(self, goal_handle, error_msg):
        self.get_logger().error(f"Request aborted: {error_msg}")
        goal_handle.abort()
        return self._assemble_result(False, error_msg, "")
    
    def _handle_cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Request canceled")
        goal_handle.canceled()
        return self._assemble_result(False, "Canceled", "")
        

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