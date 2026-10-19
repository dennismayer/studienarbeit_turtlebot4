from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

package = "speaking_service"
    
config = os.path.join(
    get_package_share_directory(package), "config", "ros_params.yaml"
)

def _create_node(executable: str, name: str, namespace: str = ""):
    return Node(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        parameters=[config],
        respawn=True,
        respawn_delay=5.0,
        respawn_max_retries=3,
    )

def generate_launch_description():
    audio_creation_node = _create_node(
        executable="audioCreationWorker",
        name="AudioCreationWorker",
        namespace="AudioWorkers",
    )

    audio_player_node = _create_node(
        executable="audioPlayerWorker",
        name="AudioPlayerWorker",
        namespace="AudioWorkers",
    )

    audio_listener_node = _create_node(
        executable="audioListenerWorker",
        name="AudioListenerWorker",
        namespace="AudioWorkers",
    )

    audio_transcriber_node = _create_node(
        executable="audioTranscriberWorker",
        name="AudioTranscriberWorker",
        namespace="AudioWorkers",
    )

    audio_orchestrator_node = _create_node(
        executable="audioOrchestratorNode",
        name="AudioOrchestrator",
    )

    return LaunchDescription([
        audio_creation_node,
        audio_player_node,
        audio_listener_node,
        audio_transcriber_node,
        audio_orchestrator_node
    ])
      
if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()