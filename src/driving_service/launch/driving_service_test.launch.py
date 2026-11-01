# Created with Claude (Anthropic) on the basis of the car_museum_guide package.
#
# Test stand-ins for driving_service.launch.py, for a robot that must not move:
# the FakeNavigator answers MoveToLocation goals without driving, and the
# ButtonVisitorTrigger starts a visit on a faceplate button press instead of
# the LiDAR. Don't run it together with driving_service.launch.py: both serve
# the same action names.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

package = 'driving_service'

config = os.path.join(
    get_package_share_directory(package), 'config', 'ros_params.yaml'
)


def _create_node(executable: str, name: str):
    return Node(
        package=package,
        executable=executable,
        name=name,
        parameters=[config],
        respawn=True,
        respawn_delay=5.0,
        respawn_max_retries=3,
    )


def generate_launch_description():
    return LaunchDescription([
        _create_node(executable='fakeNavigatorNode', name='FakeNavigator'),
        _create_node(executable='buttonVisitorTriggerNode', name='ButtonVisitorTrigger'),
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
