# Starts the whole receptionist bot as a test system, for a robot that must not move.
#
# Same as real_system.launch.py, but without localization and Nav2, and with the
# driving_service test stand-ins: the FakeNavigator reports every drive as
# reached after a few seconds, and a press of faceplate button 1 starts a visit
# instead of the LiDAR visitor detection.
#
#   ros2 launch robot_bring_up test_system.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(package: str, launch_file: str):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', launch_file)))


def generate_launch_description():
    return LaunchDescription([
        _include('speaking_service', 'speaking_service.launch.py'),
        _include('server_connector', 'server_connector.launch.py'),
        _include('driving_service', 'driving_service_test.launch.py'),
        _include('receptionist_bot', 'receptionist_bot_node.launch.py'),
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
