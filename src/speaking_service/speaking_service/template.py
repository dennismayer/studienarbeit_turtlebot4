import time

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from speaking_interfaces.action import AudioCreation

import constants

class AudioCreationActionServer(Node):
    
    def __init__(self):
        super().__init__(constants.AUDIOCREATIONNODENAME, namespace=constants.NAMESPACEWORKERS)
        self.get_logger().info("Starting Server ...")
        
        # using a Reentrant Callback Group because audio creation can be done in parallel 
        # at least theoretically -> should not appear in real life
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioCreation, 
            action_name=constants.AUDIOCREATIONACTIONNAME,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Server started!")
        
    def execute_callback(self, goal_handle: ServerGoalHandle) -> AudioCreation.Result:
        self.get_logger().info("Executing goal ...")
        request: AudioCreation.Goal = goal_handle.request
        self.get_logger().info(f"Input: {request.text}")
        
        feedback_msg = AudioCreation.Feedback()
        feedback_msg.status = 0
        goal_handle.publish_feedback(feedback_msg)
        
        for i in range(1,11):
            feedback_msg.status = i
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        
        goal_handle.succeed()
        result = AudioCreation.Result()
        result.file_name = "Geschafft!"
        return result

def main(args=None):
    rclpy.init(args=args)
    
    audio_creation_action_server = AudioCreationActionServer()
    
    executor = MultiThreadedExecutor()
    
    try:
        executor.add_node(audio_creation_action_server)
        executor.spin()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Exception occurred in main thread: {e}")
    
    executor.shutdown()
    audio_creation_action_server.destroy_node()
    
    try:
        rclpy.shutdown()
    except:
        pass
    
if __name__ == "__main__":
    main()
