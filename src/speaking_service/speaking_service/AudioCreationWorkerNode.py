from piper import PiperVoice, SynthesisConfig
import os
from pathlib import Path
import soundfile as sf
import sys

import rclpy
from rclpy.parameter import Parameter
from rclpy.action.server import ActionServer, ServerGoalHandle, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, ParameterType

from speaking_interfaces.action import AudioCreation

from speaking_service.scripts.cache import FileCache
from robot_common.parameter_helper import get_param

# default parameter in case no launch file is being used
DEFAULT_NODE_NAME = "AudioCreationWorker"
DEFAULT_NAMESPACE = "/AudioWorkers"


class AudioCreationActionServer(Node):
    """Action Server for synthesising audio"""
    def __init__(self):        
        super().__init__(DEFAULT_NODE_NAME, namespace=DEFAULT_NAMESPACE)
        self._declare_node_parameter()
        self._load_audio_voice()
        self._set_node_variables()
        self._start_server()
    
    def _declare_node_parameter(self) -> None:
        """Loads parameters for AudioCreation Node"""
        self.declare_parameters('', [
            ("action_name", "audioCreation",
                ParameterDescriptor(
                    description="Action server name, resolved inside the node namespace", 
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("voice_model_path", Parameter.Type.STRING,
                ParameterDescriptor(
                    description="Absolute path to the Piper voice model (.onnx)", 
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("volume", 1.0,
                ParameterDescriptor(
                    description="Piper synthesis volume multiplier (1.0 = unchanged)", 
                    read_only=True, 
                    type=ParameterType.PARAMETER_DOUBLE,
                    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)])),
            ("audio_cache_path", "speaking_service/audio_cache",
                ParameterDescriptor(
                    description="Audio cache directory, relative to $ROS_HOME", 
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("voice_sample_rate", 22050,
                ParameterDescriptor(
                    description="Native sample rate of the voice model", 
                    read_only=True,
                    type=ParameterType.PARAMETER_INTEGER)),
            ("voice_channels", 1,
                ParameterDescriptor(
                    description="Native channel count of the voice model", 
                    read_only=True,
                    type=ParameterType.PARAMETER_INTEGER)),
        ])
    
    def _load_audio_voice(self) -> None:
        """Loads voice for synthesising"""
        try:
            # loads voice defined in config
            self.__voice = PiperVoice.load(get_param(self, "voice_model_path", str))
            self.get_logger().info("Voice loaded!")
        except Exception as e:
            self.get_logger().error(f"Error while loading voice: {e}")
            raise RuntimeError(f"Error while loading voice: {e}")
    
    def _set_node_variables(self) -> None:
        """Reads the config parameters and directory location"""
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self.__audio_dir = ros_home / get_param(self, "audio_cache_path", str)
        self.__audio_dir.mkdir(parents=True, exist_ok=True)

        self.__sr = get_param(self, "voice_sample_rate", int)
        self.__ch = get_param(self, "voice_channels", int)
        self.__volume = get_param(self, "volume", float)
        
        self.get_logger().info("Parameters loaded!")
    
    def _start_server(self) -> None:
        # Start the server
        self.get_logger().info("Starting Server ...")
        # using a Reentrant Callback Group because audio creation can be done in parallel 
        # at least theoretically -> should not appear in real life
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioCreation, 
            action_name=get_param(self, "action_name", str),
            execute_callback=self.execute_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Server started!")
        
    def execute_callback(self, goal_handle: ServerGoalHandle) -> AudioCreation.Result:
        delete_file = True
        file_path = None
        
        try:
            self.get_logger().info("Executing goal ...")
            request: AudioCreation.Goal = goal_handle.request
            self.get_logger().info(f"Text to Transform: {request.text}")
            
            # check for empty text string
            if not request.text or request.text.strip() == "":
                self.get_logger().error("Text is empty!")
                goal_handle.abort()
                return self._assemble_result(False, "TextIsEmpty", "", 0.0)

            # declaring Synthesis parameters and defining model
            self.get_logger().info("Start TTS...")
            chunks = self.__voice.synthesize(request.text, syn_config=SynthesisConfig(volume=self.__volume))
            
            # creating temporary output file in case that the node crashes during creation
            file_path = self.__audio_dir / f'{FileCache.get_hash(request.text)}.wav.tmp'
            self.get_logger().info(f"SR: {self.__sr} Channels: {self.__ch} FilePath: {file_path}")
            
            # writing generated audio into file
            with sf.SoundFile(file_path, mode="w", samplerate=self.__sr, channels=self.__ch, format="wav") as f:
                for chunk in chunks:
                    if goal_handle.is_cancel_requested:
                        self.get_logger().info("Canceling...")
                        goal_handle.canceled()
                        return self._assemble_result(False, "Canceled", "", 0.0)
                    
                    f.write(chunk.audio_float_array)
                duration = len(f)/(self.__sr)
            
            # checks if the result contains any audio
            if duration == 0.0:
                self.get_logger().info("FileIsEmpty")
                goal_handle.abort()
                return self._assemble_result(False, "FileIsEmpty", "", 0.0)
            
            self.get_logger().info("Finished Synthesis!")
            
            # removes the .tmp ending as file creation is completed
            os.replace(file_path, file_path.with_suffix(""))
            
            # only when all the steps succeed set the flag for deleting files to False
            delete_file = False
            
            # return result
            goal_handle.succeed()
            return self._assemble_result(True, "Success", str(file_path.with_suffix("")), duration)

        except Exception as e:
            self.get_logger().error(f"Error while executing callback: {e}")
            goal_handle.abort()
            return self._assemble_result(False, f"Error while executing callback: {e}", "", 0.0)
        finally:
            # checks if file needs to be deleted
            if file_path and delete_file:
                file_path.unlink(missing_ok=True)
    
    def _assemble_result(self, success: bool, message: str, file_path: str, duration: float):
        """Assembles a result message"""
        result = AudioCreation.Result()
        result.success = success
        result.message = message
        result.file_path = file_path
        result.duration = duration
        return result

def main(args=None):
    """entry point. Node is added to executer. In case of error it will return with an error code so that a restart can be tried"""
    rclpy.init(args=args)

    audio_creation_action_server = None
    executor = None
    exit_code = 0

    try:
        audio_creation_action_server = AudioCreationActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(audio_creation_action_server)
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
        if audio_creation_action_server is not None:
            audio_creation_action_server.destroy_node()
        try_shutdown()

    return exit_code

if __name__ == "__main__":
    sys.exit(main())
