# Starts the whole receptionist bot for real operation.
#
# Localization and Nav2 come from turtlebot4_navigation and need the map the
# robot drives on. The map has no default on purpose: turtlebot4's own default
# is a sample map, and localizing on the wrong map sends the robot anywhere.
# The initial pose is still set outside (RViz "2D Pose Estimate").
#
#   ros2 launch robot_bring_up real_system.launch.py map:=/full/path/to/map.yaml
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, launch_file: str, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)),
        launch_arguments=(arguments or {}).items())


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map', description='Full path to the map yaml file the robot localizes on')

    return LaunchDescription([
        map_arg,
        _include('turtlebot4_navigation', 'localization.launch.py',
                 {'map': LaunchConfiguration('map')}),
        _include('turtlebot4_navigation', 'nav2.launch.py'),
        _include('speaking_service', 'speaking_service.launch.py'),
        _include('server_connector', 'server_connector.launch.py'),
        _include('driving_service', 'driving_service.launch.py'),
        _include('receptionist_bot', 'receptionist_bot_node.launch.py'),
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
