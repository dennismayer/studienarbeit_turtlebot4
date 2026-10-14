import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node

from speaking_interfaces.action import AudioCreation

import constants

class SpeechOrchestratorNode(Node):
    
    def __init__(self):
        super().__init__("SpeechOrchestrator")
        
        self.get_logger().info("start node")
        
        self.__audio_creation_action_client = ActionClient(
            self, 
            AudioCreation,
            constants.NAMESPACEWORKERS + "/" + constants.AUDIOCREATIONACTIONNAME
        )
        
        self.get_logger().info(constants.NAMESPACEWORKERS + "/" + constants.AUDIOCREATIONACTIONNAME)
        
    def send_goal(self, text):
        goal_msg = AudioCreation.Goal()
        goal_msg.text = text
        
        self.__audio_creation_action_client.wait_for_server()
        return self.__audio_creation_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    
    orchestrator_node = SpeechOrchestratorNode()
    
    future = orchestrator_node.send_goal("Hallo, wie geht es euch?")

    rclpy.spin_until_future_complete(orchestrator_node, future)

if __name__ == "__main__":
    main()