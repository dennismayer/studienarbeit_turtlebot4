import time
from faster_whisper import WhisperModel 
import soundfile as sf
import numpy as np
import os
from pathlib import Path
import sys

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown

from speaking_interfaces.action import AudioTranscriber

import constants

class AudioTranscriberActionServer(Node):
    
    def __init__(self):
        super().__init__(constants.AUDIOTRANSCRIBERNODENAME, namespace=constants.NAMESPACEWORKERS)
        
        self.__model= WhisperModel(
            model_size_or_path=constants.TRANSCRIPTION_MODEL_SIZE, 
            device=constants.TRANSCRIPTION_DEVICE, 
            compute_type=constants.TRANSCRIPTION_COMPUTE_TYPE,
            local_files_only=True
        )

        # only recordings from the listener may be deleted after transcription
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self.__recorded_audio_dir = (ros_home / 'speaking_service' / 'recorded_audio_cache').resolve()

        self.get_logger().info("Starting Server ...")
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioTranscriber, 
            action_name=constants.AUDIOTRANSCRIBERACTIONNAME,
            execute_callback=self.execute_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Server started!")
        
    def execute_callback(self, goal_handle: ServerGoalHandle) -> AudioTranscriber.Result:
        try:
            self.get_logger().info("Executing goal ...")
            request: AudioTranscriber.Goal = goal_handle.request
            self.get_logger().info(f"File: {request.file_path}")
            self.get_logger().info(f"Prompt: {request.initial_prompt}")
            
            # load audio
            self.get_logger().info("Loading data...")
            info = sf.info(request.file_path)
            # check sample rate and channel count
            if info.samplerate != constants.TRANSCRIPTION_SR or info.channels != constants.TRANSCRIPTION_CH:
                raise ValueError(f"Wrong sample rate ({info.samplerate} Hz) or Channel count ({info.channels})")
            
            audio, _ = sf.read(request.file_path, dtype="float32")
            
            # define Transcription 
            segments, info = self.__model.transcribe(
                audio=audio,
                language=request.language if request.language != "" else None,
                initial_prompt=request.initial_prompt if request.initial_prompt != "" else None,
                beam_size=constants.TRANSCRIPTION_BEAM_SIZE,
                temperature=constants.TRANSCRIPTION_TEMPERATURE,
                condition_on_previous_text=constants.TRANSCRIPTION_CONDITION_ON_PREVIOUS_TEXT,
                vad_filter=constants.TRANSCRIPTION_VAD_FILTER,
                vad_parameters=constants.TRANSCRIPTION_VAD_PARAMETERS,
                no_speech_threshold=constants.TRANSCRIPTION_NO_SPEECH_THRESHOLD
            )
            
            # transcribe
            word_array = []
            avg_logprob = []
            no_speech_prob = []
            
            self.get_logger().info("Transcribing...")
            for segment in segments:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Cancel request...")
                    return self.assemble_result(False, "Canceled", word_array, avg_logprob, no_speech_prob)
                
                word_array.append(segment.text.strip())
                avg_logprob.append(segment.avg_logprob)
                no_speech_prob.append(segment.no_speech_prob)
                goal_handle.publish_feedback(self.assemble_feedback(word_array, segment.end / info.duration))
            
            # send results
            self.get_logger().info("Finished!")
            goal_handle.succeed()     
            
            # delete file, but only if it lies inside the recording cache
            path_to_file = Path(request.file_path).resolve()
            if path_to_file.is_relative_to(self.__recorded_audio_dir):
                try:
                    path_to_file.unlink(missing_ok=True)
                except OSError as e:
                    self.get_logger().warning(f"Could not delete {path_to_file}: {e}")
            else:
                self.get_logger().info(f"Keeping {path_to_file}: not in {self.__recorded_audio_dir}")

            return self.assemble_result(True, "FinishedWithoutError", word_array, avg_logprob, no_speech_prob)
        
        except Exception as e:
            self.get_logger().error(f"{e}")
            goal_handle.abort()
            return self.assemble_result(False, f"{e}", [], [], [])          
    
    def assemble_result(self, success: bool, message: str, word_array: list[str], avg_logprob_array: list[float], no_speech_prob_array: list[float]):
        result = AudioTranscriber.Result()
        result.success = success
        result.message = message
        result.text = " ".join(word_array)
        result.avg_logprob = np.average(avg_logprob_array) if avg_logprob_array != [] else np.nan
        result.no_speech_prob = np.max(no_speech_prob_array) if no_speech_prob_array != [] else np.nan
        return result
    
    def assemble_feedback(self, word_array: list[str], progress):
        feedback = AudioTranscriber.Feedback()
        feedback.partial_text = " ".join(word_array)
        feedback.progress = progress
        return feedback

def main(args=None):
    rclpy.init(args=args)

    audio_transcriber_action_server = None
    executor = None

    try:
        audio_transcriber_action_server = AudioTranscriberActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(audio_transcriber_action_server)
        executor.spin()
    except RuntimeError as e:
        print(e, file=sys.stderr)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Exception occurred in main thread: {e}")
    finally:
        if executor is not None:
            executor.shutdown()
        if audio_transcriber_action_server is not None:
            audio_transcriber_action_server.destroy_node()
        try_shutdown()

if __name__ == "__main__":
    main()
