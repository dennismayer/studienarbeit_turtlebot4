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
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, ParameterType

from speaking_interfaces.action import AudioTranscriber

from speaking_service.scripts.parameter_helper import get_param

class AudioTranscriberActionServer(Node):

    def __init__(self):
        # defaults
        node_name = "AudioTranscriberWorker"
        node_namespace = "/AudioWorkers"

        super().__init__(node_name, namespace=node_namespace)

        # load parameters
        self.declare_parameters('', [
            ("action_name", "audioTranscriber",
             ParameterDescriptor(
                 description="Action server name, resolved inside the node namespace",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("model_size", "tiny",
             ParameterDescriptor(
                 description="Whisper model size (tiny, base, small, medium, large-v3, ...)",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("device", "cpu",
             ParameterDescriptor(
                 description="Inference device (cpu or cuda)",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("compute_type", "int8",
             ParameterDescriptor(
                 description="Model weight quantization (int8, float16, float32, ...)",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
            ("beam_size", 5,
             ParameterDescriptor(
                 description="Number of beams for beam search decoding",
                 read_only=True,
                 type=ParameterType.PARAMETER_INTEGER)),
            ("temperature", [0.0, 0.2, 0.4, 0.6, 0.8, 1.0],
             ParameterDescriptor(
                 description="Sampling temperatures, tried in order when decoding fails",
                 read_only=True,
                 type=ParameterType.PARAMETER_DOUBLE_ARRAY)),
            ("condition_on_previous_text", False,
             ParameterDescriptor(
                 description="Feed the previous segment's text as a prompt for the next one",
                 read_only=True,
                 type=ParameterType.PARAMETER_BOOL)),
            ("vad_filter", True,
             ParameterDescriptor(
                 description="Drop non-speech parts with the Silero VAD before transcribing",
                 read_only=True,
                 type=ParameterType.PARAMETER_BOOL)),
            ("no_speech_threshold", 0.6,
             ParameterDescriptor(
                 description="Segment dropped if no_speech_prob > this AND avg_logprob < -1.0",
                 read_only=True,
                 type=ParameterType.PARAMETER_DOUBLE,
                 floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)])),
            ("recording_sample_rate", 16000,
             ParameterDescriptor(
                 description="Expected sample rate of the recordings to transcribe",
                 read_only=True,
                 type=ParameterType.PARAMETER_INTEGER)),
            ("recording_channels", 1,
             ParameterDescriptor(
                 description="Expected channel count of the recordings to transcribe",
                 read_only=True,
                 type=ParameterType.PARAMETER_INTEGER)),
            ("recorded_audio_cache_path", "speaking_service/recorded_audio_cache",
             ParameterDescriptor(
                 description="Recording cache directory, relative to $ROS_HOME",
                 read_only=True,
                 type=ParameterType.PARAMETER_STRING)),
        ])
        # VAD options, declared as vad_parameters.<key>; other keys keep faster-whisper defaults
        self.declare_parameters('vad_parameters', [
            ("min_silence_duration_ms", 1000,
             ParameterDescriptor(
                 description="Minimum silence in ms that splits speech into separate segments",
                 read_only=True,
                 type=ParameterType.PARAMETER_INTEGER)),
        ])

        self.__model= WhisperModel(
            model_size_or_path=get_param(self, "model_size", str),
            device=get_param(self, "device", str),
            compute_type=get_param(self, "compute_type", str),
            local_files_only=True
        )

        # Parameter definition
        self.__sr = get_param(self, "recording_sample_rate", int)
        self.__ch = get_param(self, "recording_channels", int)

        self.__beam_size = get_param(self, "beam_size", int)
        self.__temperature = get_param(self, "temperature", list)
        self.__condition_on_previous_text = get_param(self, "condition_on_previous_text", bool)
        self.__vad_filter = get_param(self, "vad_filter", bool)
        self.__vad_parameters = {
            "min_silence_duration_ms": get_param(self, "vad_parameters.min_silence_duration_ms", int)
        }
        self.__no_speech_threshold = get_param(self, "no_speech_threshold", float)

        # only recordings from the listener may be deleted after transcription
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self.__recorded_audio_dir = (ros_home / get_param(self, "recorded_audio_cache_path", str)).resolve()

        self.get_logger().info("Starting Server ...")
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioTranscriber, 
            action_name=get_param(self, "action_name", str),
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
            if info.samplerate != self.__sr or info.channels != self.__ch:
                raise ValueError(f"Wrong sample rate ({info.samplerate} Hz) or Channel count ({info.channels})")
            
            audio, _ = sf.read(request.file_path, dtype="float32")
            
            # define Transcription 
            segments, info = self.__model.transcribe(
                audio=audio,
                language=request.language if request.language != "" else None,
                initial_prompt=request.initial_prompt if request.initial_prompt != "" else None,
                beam_size=self.__beam_size,
                temperature=self.__temperature,
                condition_on_previous_text=self.__condition_on_previous_text,
                vad_filter=self.__vad_filter,
                vad_parameters=self.__vad_parameters,
                no_speech_threshold=self.__no_speech_threshold
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
    exit_code = 0

    try:
        audio_transcriber_action_server = AudioTranscriberActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(audio_transcriber_action_server)
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
        if audio_transcriber_action_server is not None:
            audio_transcriber_action_server.destroy_node()
        try_shutdown()

    return exit_code

if __name__ == "__main__":
    sys.exit(main())
