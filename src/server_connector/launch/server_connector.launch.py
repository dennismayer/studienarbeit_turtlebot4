from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

package = "server_connector"

config = os.path.join(
    get_package_share_directory(package), "config", "ros_params.yaml"
)

# log level of this package's nodes, e.g. log_level:=debug; only their own
# loggers change, so rcl and DDS stay quiet
log_level_arg = DeclareLaunchArgument(
    "log_level", default_value="info", description="Log level of the nodes in this package"
)

def _create_node(executable: str, name: str, namespace: str = ""):
    logger = f"{namespace}.{name}" if namespace else name
    return Node(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        parameters=[config],
        ros_arguments=["--log-level", [logger, ":=", LaunchConfiguration("log_level")]],
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
        log_level_arg,
        server_connector_node
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
