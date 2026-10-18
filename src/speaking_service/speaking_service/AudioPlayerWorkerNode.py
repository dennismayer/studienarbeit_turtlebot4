import sys
import sounddevice as sd
import soundfile as sf
from scipy import signal
import numpy as np
from threading import Lock

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown

from speaking_interfaces.action import AudioPlayer

import constants

class AudioPlayerActionServer(Node):
    
    def __init__(self):
        super().__init__(constants.AUDIOPLAYERNODENAME, namespace=constants.NAMESPACEWORKERS)
        
        # Parameter definition
        self.lock_player_node = Lock()
        self.__sr = constants.VOICE_SAMPLE_RATE
        self.__ch = constants.VOICE_CHANNELS

        self.stream = sd.OutputStream(
            samplerate=self.__sr,
            channels=self.__ch,
            dtype="float32",
            latency="high",
            device=constants.AUDIO_DEVICE
        )
        
        self.stream.start()
        
        self.get_logger().info("Starting Server ...")
        
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioPlayer, 
            action_name=constants.AUDIOPLAYERACTIONNAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_logger().info("Server started!")
         
        
    def execute_callback(self, goal_handle: ServerGoalHandle) -> AudioPlayer.Result:
        duration_played = 0.0
        
        try:
            self.get_logger().info("Executing goal ...")
            request: AudioPlayer.Goal = goal_handle.request
            self.get_logger().info(f"Data path: {request.file_path}")
            
            result = AudioPlayer.Result()
            
            self.get_logger().info("Start reading data...")
            
            data, sr = sf.read(request.file_path, dtype="float32", always_2d=True)
            
            if sr != constants.CURRENT_VOICE_SAMPLE_RATE or data.shape[1] != constants.CURRENT_VOICE_CHANNELS:
                raise ValueError(f"Format mismatch: {sr} Hz/{data.shape[1]} ch")
            
            # transform input data
            self.get_logger().info("Transforming data...")
            data = signal.resample_poly(data, up=2, down=1)
            data = np.hstack([data, data.copy()])

                        
            self.get_logger().info("Start playing data...")
            chunk = int(self.__sr * constants.CHUNK_S)
            feedback = AudioPlayer.Feedback()
            for start in range(0, len(data), chunk):
                if goal_handle.is_cancel_requested:
                    self.stream.abort()
                    self.stream.start()
                    goal_handle.canceled()
                    result.success, result.message, result.played_time = False, "Canceled", duration_played
                    return result

                self.stream.write(data[start:start + chunk])
                
                total_chunks_played = min(start+chunk, len(data))
                
                duration_played = total_chunks_played / self.__sr
                feedback.remaining_time = (len(data) - total_chunks_played) / self.__sr
                feedback.progress = total_chunks_played / len(data)
                goal_handle.publish_feedback(feedback)

            self.get_logger().info("Finished playback!")
            goal_handle.succeed()
            result.success, result.message, result.played_time = True, "Done", duration_played
            return result
        
        except Exception as e:
            self.get_logger().error(f"{e}")
            goal_handle.abort()
            result = AudioPlayer.Result()
            result.success, result.message, result.played_time = False, f"{e}", duration_played
            return result
        finally:
            self.lock_player_node.release()
    
    def goal_callback(self, goal_request):
        if not self.lock_player_node.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def destroy_node(self):
        self.stream.stop()
        self.stream.close()
        super().destroy_node()
        

def main(args=None):
    rclpy.init(args=args)

    audio_player_action_server = None
    executor = None

    try:
        audio_player_action_server = AudioPlayerActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(audio_player_action_server)
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
        if audio_player_action_server is not None:
            audio_player_action_server.destroy_node()
        try_shutdown()

if __name__ == "__main__":
    main()
