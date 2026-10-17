import time
from pathlib import Path
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
        self.get_logger().info("Executing goal ...")
        request: AudioPlayer.Goal = goal_handle.request
        self.get_logger().info(f"Data path: {request.file_name}")
        
        result = AudioPlayer.Result()
        
        self.get_logger().info("Start reading data...")
        
        try:
            data, sr = sf.read(request.file_name, dtype="float32", always_2d=True)
        except Exception as e:
            return self.handle_exception_callback(goal_handle, f"Failed to read file: {e}")
        
        if sr != constants.CURRENT_VOICE_SAMPLE_RATE or data.shape[1] != constants.CURRENT_VOICE_CHANNELS:
            return self.handle_exception_callback(goal_handle, f"Format mismatch: {sr} Hz/{data.shape[1]} ch")
        
        self.get_logger().info("Transforming data...")
        # transform input data
        try:
            data = signal.resample_poly(data, up=2, down=1)
            data = np.hstack([data, data.copy()])
        except Exception as e:
            return self.handle_exception_callback(goal_handle, "Error in Transformation!")
                    
        self.get_logger().info("Start playing data...")
        chunk = int(self.__sr * constants.CHUNK_S)
        feedback = AudioPlayer.Feedback()
        try:
            for start in range(0, len(data), chunk):
                if goal_handle.is_cancel_requested:
                    self.stream.abort()
                    self.stream.start()
                    goal_handle.canceled()
                    result.success, result.message = False, "Cancelled"
                    return result

                self.stream.write(data[start:start + chunk])
                feedback.remaining_time = max(0.0, (len(data) - start - chunk) / self.__sr)
                goal_handle.publish_feedback(feedback)
        finally:
            self.lock_player_node.release()

        self.get_logger().info("Finished playback!")
        goal_handle.succeed()
        result.success, result.message = True, "Done"
        return result
    
    def handle_exception_callback(self, goal_handle, err_message):
        self.get_logger().error(err_message)
        goal_handle.abort()
        self.lock_player_node.release()
        result = AudioPlayer.Result()
        result.success, result.message = False, err_message
        return result
    
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
    
    try:
        audio_player_action_server = AudioPlayerActionServer()
    except RuntimeError as e:
        print(e,file=sys.stderr)
        return
    
    executor = MultiThreadedExecutor()
    
    try:
        executor.add_node(audio_player_action_server)
        executor.spin()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Exception occurred in main thread: {e}")
    
    executor.shutdown()
    audio_player_action_server.destroy_node()
    
    try:
        rclpy.shutdown()
    except:
        pass
    
if __name__ == "__main__":
    main()
