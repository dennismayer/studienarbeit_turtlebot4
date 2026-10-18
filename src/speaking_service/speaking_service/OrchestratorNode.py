import sys

import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown

from speaking_interfaces.action import AudioCreation, AudioListener, AudioPlayer, AudioTranscriber

import constants

class SpeechOrchestratorNode(Node):
    
    def __init__(self):
        super().__init__("SpeechOrchestrator")
        
        self.get_logger().info("Start Action Clients ...")
        
        self.__audio_creation_action_client = ActionClient(
            self, 
            AudioCreation,
            constants.NAMESPACEWORKERS + "/" + constants.AUDIOCREATIONACTIONNAME
        )
        
        self.__audio_player_action_client = ActionClient(
            self, 
            AudioPlayer,
            constants.NAMESPACEWORKERS + "/" + constants.AUDIOPLAYERACTIONNAME
        )
        
        self.__audio_listener_action_client = ActionClient(
            self, 
            AudioListener,
            constants.NAMESPACEWORKERS + "/" + constants.AUDIOLISTENERACTIONNAME
        )

        self.__audio_transcriber_action_client = ActionClient(
            self, 
            AudioTranscriber,
            constants.NAMESPACEWORKERS + "/" + constants.AUDIOTRANSCRIBERACTIONNAME
        )
        
        self.get_logger().info("Action Clients started!")
        
    def send_goal(self, text):
        goal_msg = AudioCreation.Goal()
        goal_msg.text = text
        
        self.__audio_creation_action_client.wait_for_server()
        return self.__audio_creation_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    orchestrator_node = None
    executor = None

    try:
        orchestrator_node = SpeechOrchestratorNode()
        executor = MultiThreadedExecutor()
        executor.add_node(orchestrator_node)

        # test goal
        orchestrator_node.send_goal("Hallo, wie geht es euch?")

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
        if orchestrator_node is not None:
            orchestrator_node.destroy_node()
        try_shutdown()

if __name__ == "__main__":
    main()