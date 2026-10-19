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
from speaking_service.scripts.parameter_helper import get_param

class AudioCreationActionServer(Node):
    
    def __init__(self):
        # default vars
        node_name = "AudioCreationWorker"
        node_namespace = "/AudioWorkers"
        
        super().__init__(node_name, namespace=node_namespace)
        
        # load parameters
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
        
        try:
            # load voice 
            self.__voice = PiperVoice.load(get_param(self, "voice_model_path", str))
            self.get_logger().info("Voice loaded!")
        except Exception as e:
            self.get_logger().error(f"Error while loading voice: {e}")
            raise RuntimeError(f"Error while loading voice: {e}")
            
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self.__audio_dir = ros_home / get_param(self, "audio_cache_path", str)
        self.__audio_dir.mkdir(parents=True, exist_ok=True)

        self.__sr = get_param(self, "voice_sample_rate", int)
        self.__ch = get_param(self, "voice_channels", int)
        self.__volume = get_param(self, "volume", float)
                
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
            
            # abort if text is empty
            if not request.text or request.text.strip() == "":
                self.get_logger().error("Text is empty!")
                goal_handle.abort()
                return self.assemble_result(False, "TextIsEmpty", "", 0.0)
            
            self.get_logger().info("Start TTS...")
            chunks = self.__voice.synthesize(request.text, syn_config=SynthesisConfig(volume=self.__volume))
            
            file_path = self.__audio_dir / f'{FileCache.get_hash(request.text)}.wav.tmp'
            self.get_logger().info(f"SR: {self.__sr} Channels: {self.__ch} FilePath: {file_path}")
              
            with sf.SoundFile(file_path, mode="w", samplerate=self.__sr, channels=self.__ch, format="wav") as f:
                for chunk in chunks:
                    if goal_handle.is_cancel_requested:
                        self.get_logger().info("Canceling...")
                        goal_handle.canceled()
                        return self.assemble_result(False, "Canceled", "", 0.0)
                    
                    f.write(chunk.audio_float_array)
                duration = len(f)/(self.__sr)
            
            if duration == 0.0:
                self.get_logger().info("FileIsEmpty")
                goal_handle.abort()
                return self.assemble_result(False, "FileIsEmpty", "", 0.0)
            
            self.get_logger().info("Finished Synthesis!")
            os.replace(file_path, file_path.with_suffix(""))
            delete_file = False
            goal_handle.succeed()
            return self.assemble_result(True, "Success", str(file_path.with_suffix("")), duration)

        except Exception as e:
            self.get_logger().error(f"Error while executing callback: {e}")
            goal_handle.abort()
            return self.assemble_result(False, f"Error while executing callback: {e}", "", 0.0)
        finally:
            if file_path and delete_file:
                file_path.unlink(missing_ok=True)
    
    def assemble_result(self, success: bool, message: str, file_path: str, duration: float):
        result = AudioCreation.Result()
        result.success = success
        result.message = message
        result.file_path = file_path
        result.duration = duration
        return result

def main(args=None):
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
