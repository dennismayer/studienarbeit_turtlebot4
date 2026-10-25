"""
MoveToLocation action server on top of Nav2.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

import math
import sys
import threading
import time

from action_msgs.msg import GoalStatus
from driving_interfaces.action import MoveToLocation
from driving_service.run_node import run
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class NavigatorNode(Node):
    """Drives to a map pose by sending navigate_to_pose goals to Nav2."""

    def __init__(self, **kwargs):
        super().__init__('Navigator', **kwargs)
        self.declare_parameters('', [
            ('action_name', 'moveToLocation', ParameterDescriptor(
                description='Name of the MoveToLocation action server',
                read_only=True, type=ParameterType.PARAMETER_STRING)),
            ('nav2_action', 'navigate_to_pose', ParameterDescriptor(
                description="Name of Nav2's NavigateToPose action server",
                read_only=True, type=ParameterType.PARAMETER_STRING)),
            ('nav_retry_attempts', 3, ParameterDescriptor(
                description='Nav2 attempts per goal, including the first one',
                read_only=True, type=ParameterType.PARAMETER_INTEGER)),
            ('nav_retry_delay', 5.0, ParameterDescriptor(
                description='Seconds to wait before retrying a failed Nav2 goal',
                read_only=True, type=ParameterType.PARAMETER_DOUBLE)),
            ('nav2_server_timeout', 5.0, ParameterDescriptor(
                description='Seconds to wait for the Nav2 action server per goal',
                read_only=True, type=ParameterType.PARAMETER_DOUBLE)),
        ])
        self._busy = False
        self._busy_lock = threading.Lock()
        self._nav_handle = None

        # reentrant, so Nav2 responses arrive while a goal is being executed
        group = ReentrantCallbackGroup()
        self._nav2 = ActionClient(
            self, NavigateToPose, self.get_parameter('nav2_action').value,
            callback_group=group)
        self._server = ActionServer(
            self, MoveToLocation, self.get_parameter('action_name').value,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._execute,
            callback_group=group)

    def _on_goal(self, _goal):
        with self._busy_lock:
            if self._busy:
                self.get_logger().warning('Rejected goal: already driving')
                return GoalResponse.REJECT
            self._busy = True
            return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        nav_handle = self._nav_handle
        if nav_handle is not None:
            nav_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._drive(goal_handle)
        finally:
            with self._busy_lock:
                self._busy = False

    def _drive(self, goal_handle):
        request = goal_handle.request
        server_timeout = self.get_parameter('nav2_server_timeout').value
        retry_delay = self.get_parameter('nav_retry_delay').value
        if not self._nav2.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error('Nav2 action server is not available')
            goal_handle.abort()
            return MoveToLocation.Result(success=False, message='Nav2Unavailable', attempts=0)
        max_attempts = self.get_parameter('nav_retry_attempts').value
        result = MoveToLocation.Result()
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                self._sleep_unless_canceled(goal_handle, retry_delay)
                if goal_handle.is_cancel_requested:
                    result.message = 'Canceled'
                    goal_handle.canceled()
                    return result
            result.attempts = attempt
            nav_goal = NavigateToPose.Goal(pose=self._map_pose(request.x, request.y, request.yaw))
            nav_handle = self._wait(self._nav2.send_goal_async(
                nav_goal,
                feedback_callback=lambda msg, a=attempt: self._forward_feedback(
                    goal_handle, a, msg)))
            if not nav_handle.accepted:
                result.message = 'Nav2 rejected the goal'
                self.get_logger().warning(f'Attempt {attempt}/{max_attempts}: {result.message}')
                continue
            self._nav_handle = nav_handle
            if goal_handle.is_cancel_requested:
                # the cancel arrived before Nav2 accepted, _on_cancel had nothing to forward
                nav_handle.cancel_goal_async()
            nav_result = self._wait(nav_handle.get_result_async())
            self._nav_handle = None
            if goal_handle.is_cancel_requested:
                result.message = 'Canceled'
                goal_handle.canceled()
                return result
            if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
                result.success = True
                result.message = 'Reached'
                goal_handle.succeed()
                return result
            error = nav_result.result
            result.message = f'Nav2 error {error.error_code}: {error.error_msg}'
            self.get_logger().warning(f'Attempt {attempt}/{max_attempts} failed: {result.message}')
        goal_handle.abort()
        return result

    def _forward_feedback(self, goal_handle, attempt, nav_feedback_msg):
        goal_handle.publish_feedback(MoveToLocation.Feedback(
            attempt=attempt,
            distance_remaining=nav_feedback_msg.feedback.distance_remaining))

    def _sleep_unless_canceled(self, goal_handle, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not goal_handle.is_cancel_requested:
            if not self.context.ok():
                raise RuntimeError('Shutting down')
            time.sleep(0.05)

    def _wait(self, future):
        """Block until a future is done; needs a multi-threaded executor."""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        while not done.wait(0.1):
            if not self.context.ok():
                raise RuntimeError('Shutting down')
        return future.result()

    def _map_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def main(args=None):
    """Run the node."""
    return run(NavigatorNode, args)


if __name__ == '__main__':
    sys.exit(main())
