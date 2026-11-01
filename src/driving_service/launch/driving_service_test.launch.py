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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package = 'driving_service'

config = os.path.join(
    get_package_share_directory(package), 'config', 'ros_params.yaml'
)


# log level of the test nodes, e.g. log_level:=debug; only their own loggers change
log_level_arg = DeclareLaunchArgument(
    'log_level', default_value='info', description='Log level of the test nodes')


def _create_node(executable: str, name: str):
    return Node(
        package=package,
        executable=executable,
        name=name,
        parameters=[config],
        ros_arguments=['--log-level', [name, ':=', LaunchConfiguration('log_level')]],
        respawn=True,
        respawn_delay=5.0,
        respawn_max_retries=3,
    )


def generate_launch_description():
    return LaunchDescription([
        log_level_arg,
        _create_node(executable='fakeNavigatorNode', name='FakeNavigator'),
        _create_node(executable='buttonVisitorTriggerNode', name='ButtonVisitorTrigger'),
    ])


if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
