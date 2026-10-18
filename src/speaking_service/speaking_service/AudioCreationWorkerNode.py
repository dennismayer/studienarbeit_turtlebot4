from piper import PiperVoice, SynthesisConfig
import os
import uuid
from pathlib import Path
import soundfile as sf
import sys

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown

from speaking_interfaces.action import AudioCreation

import constants

class AudioCreationActionServer(Node):
    
    def __init__(self):
        super().__init__(constants.AUDIOCREATIONNODENAME, namespace=constants.NAMESPACEWORKERS)
        
        try:
            # load voice 
            self.__voice = PiperVoice.load(constants.VOICE_PATH)
            self.get_logger().info("Voice loaded!")
        except Exception as e:
            self.get_logger().error(f"Error while loading voice: {e}")
            raise RuntimeError(f"Error while loading voice: {e}")
            
        
        # TODO write in constants or even as parameter?
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self.__audio_dir = ros_home / 'speaking_service' / 'audio_cache'
        self.__audio_dir.mkdir(parents=True, exist_ok=True)
        self.__sr = constants.CURRENT_VOICE_SAMPLE_RATE
        self.__ch = constants.CURRENT_VOICE_CHANNELS
                
        # Start the server
        self.get_logger().info("Starting Server ...")
        # using a Reentrant Callback Group because audio creation can be done in parallel 
        # at least theoretically -> should not appear in real life
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioCreation, 
            action_name=constants.AUDIOCREATIONACTIONNAME,
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
            chunks = self.__voice.synthesize(request.text, syn_config=SynthesisConfig(volume=constants.VOLUME))
            
            file_path = self.__audio_dir / f'{uuid.uuid4().hex}.wav'
            self.get_logger().info(f"SR: {self.__sr} Channels: {self.__ch} FilePath: {file_path}")
              
            with sf.SoundFile(file_path, mode="x", samplerate=self.__sr, channels=self.__ch) as f:
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
            delete_file = False
            goal_handle.succeed()
            return self.assemble_result(True, "Success", str(file_path), duration)

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

    try:
        audio_creation_action_server = AudioCreationActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(audio_creation_action_server)
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
        if audio_creation_action_server is not None:
            audio_creation_action_server.destroy_node()
        try_shutdown()

if __name__ == "__main__":
    main()
