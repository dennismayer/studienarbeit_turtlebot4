from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

package = "server_connector"

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
    server_connector_node = _create_node(
        executable="serverConnectorNode",
        name="ServerConnectorNode",
    )

    return LaunchDescription([
        server_connector_node
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
