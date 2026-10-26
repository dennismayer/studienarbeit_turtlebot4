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
from driving_service.action_peer import server_gone, server_identity
from driving_service.run_node import run
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


# seconds between checks whether Nav2 is still the server that got the goal
PEER_CHECK_PERIOD = 0.5
# seconds Ctrl-C waits for Nav2 to confirm the cancel of the running goal
STOP_CANCEL_TIMEOUT = 2.0
# longest Ctrl-C waits for Nav2 to answer a goal request that is still pending
STOP_PENDING_MAX = 5.0
# seconds Ctrl-C keeps spinning after the running goal ended, so its result goes out
STOP_RESULT_FLUSH = 0.5


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
            ('nav2_response_timeout', 5.0, ParameterDescriptor(
                description='Seconds to wait for Nav2 to accept or reject a goal',
                read_only=True, type=ParameterType.PARAMETER_DOUBLE)),
        ])
        self._check_parameters()
        self._busy = False
        self._busy_lock = threading.Lock()
        self._nav_handle = None
        self._send_future = None  # goal request to Nav2 that has no answer yet
        self._send_deadline = 0.0  # monotonic time its nav2_response_timeout runs out
        self._stopping = threading.Event()  # Ctrl-C: end the running goal, start no new attempt

        # reentrant, so Nav2 responses arrive while a goal is being executed
        group = ReentrantCallbackGroup()
        self._nav2_name = self.get_parameter('nav2_action').value
        self._nav2 = ActionClient(
            self, NavigateToPose, self._nav2_name,
            callback_group=group)
        self._server = ActionServer(
            self, MoveToLocation, self.get_parameter('action_name').value,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._execute,
            callback_group=group)

    def _check_parameters(self):
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        checks = [
            (value('nav_retry_attempts') >= 1, 'nav_retry_attempts must be >= 1'),
            (value('nav_retry_delay') >= 0.0, 'nav_retry_delay must be >= 0'),
            (value('nav2_server_timeout') > 0.0, 'nav2_server_timeout must be > 0'),
            (value('nav2_response_timeout') > 0.0, 'nav2_response_timeout must be > 0'),
        ]
        for ok, message in checks:
            if not ok:
                raise ValueError(message)

    def _on_goal(self, goal):
        if self._stopping.is_set():
            self.get_logger().warning('Rejected goal: shutting down')
            return GoalResponse.REJECT
        if not all(math.isfinite(v) for v in (goal.x, goal.y, goal.yaw)):
            self.get_logger().warning(f'Rejected goal with a non-finite coordinate: {goal}')
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                self.get_logger().warning('Rejected goal: already driving')
                return GoalResponse.REJECT
            self._busy = True
            return GoalResponse.ACCEPT

    def stop(self, executor):
        """
        Cancel the running Nav2 goal, so Nav2 doesn't drive on without us.

        The running MoveToLocation goal then ends with "Navigator shutting down".
        """
        # read before _stopping is set: from then on _drive may give up and forget it
        send_future = self._send_future
        self._stopping.set()
        if send_future is not None:
            # Nav2 may still accept the goal we just sent; wait for its answer as long as
            # the response timeout would have (at most STOP_PENDING_MAX, ros2 launch kills
            # after 10 s), so a late acceptance is canceled, not orphaned
            remaining = self._send_deadline - time.monotonic()
            executor.spin_until_future_complete(
                send_future, timeout_sec=min(max(0.0, remaining), STOP_PENDING_MAX))
            if send_future.done() and send_future.result().accepted:
                self._nav_handle = send_future.result()
        nav_handle = self._nav_handle
        if nav_handle is not None:
            self.get_logger().info('Canceling the running Nav2 goal')
            executor.spin_until_future_complete(
                nav_handle.cancel_goal_async(), timeout_sec=STOP_CANCEL_TIMEOUT)
        # let the running goal end, and rclpy send its result, before ROS goes down
        deadline = time.monotonic() + STOP_CANCEL_TIMEOUT
        while self._busy and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        flush_until = time.monotonic() + STOP_RESULT_FLUSH
        while time.monotonic() < flush_until:
            executor.spin_once(timeout_sec=0.05)

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
        response_timeout = self.get_parameter('nav2_response_timeout').value
        max_attempts = self.get_parameter('nav_retry_attempts').value
        result = MoveToLocation.Result()
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                self._sleep_unless_canceled(goal_handle, retry_delay)
                if goal_handle.is_cancel_requested:
                    result.message = 'Canceled'
                    goal_handle.canceled()
                    return result
                if self._stopping.is_set():
                    return self._end_as_stopping(goal_handle, result)
            if not self._wait_for_nav2(goal_handle, server_timeout):
                if self._stopping.is_set():
                    return self._end_as_stopping(goal_handle, result)
                if goal_handle.is_cancel_requested:
                    result.message = 'Canceled'
                    goal_handle.canceled()
                    return result
                self.get_logger().error('Nav2 action server is not available')
                result.message = 'Nav2Unavailable'
                goal_handle.abort()
                return result
            nav2 = server_identity(self, self._nav2_name)
            result.attempts = attempt
            nav_goal = NavigateToPose.Goal(pose=self._map_pose(request.x, request.y, request.yaw))
            send_future = self._nav2.send_goal_async(
                nav_goal,
                feedback_callback=lambda msg, a=attempt: self._forward_feedback(
                    goal_handle, a, msg))
            self._send_future = send_future
            self._send_deadline = time.monotonic() + response_timeout
            nav_handle = self._wait(
                send_future, timeout=response_timeout, give_up=self._stopping.is_set)
            self._send_future = None
            if nav_handle is None:
                # the request or its answer got lost; never leave a late acceptance driving
                send_future.add_done_callback(_cancel_if_accepted)
                if self._stopping.is_set():
                    return self._end_as_stopping(goal_handle, result)
                result.message = 'Nav2 did not answer the goal request'
                self.get_logger().warning(f'Attempt {attempt}/{max_attempts}: {result.message}')
                continue
            if not nav_handle.accepted:
                result.message = 'Nav2 rejected the goal'
                self.get_logger().warning(f'Attempt {attempt}/{max_attempts}: {result.message}')
                continue
            self._nav_handle = nav_handle
            if goal_handle.is_cancel_requested:
                # the cancel arrived before Nav2 accepted, _on_cancel had nothing to forward
                nav_handle.cancel_goal_async()
            nav_result = self._wait(
                nav_handle.get_result_async(),
                give_up=lambda: server_gone(self, self._nav2_name, nav2))
            self._nav_handle = None
            if goal_handle.is_cancel_requested:
                result.message = 'Canceled'
                goal_handle.canceled()
                return result
            if self._stopping.is_set():
                return self._end_as_stopping(goal_handle, result)
            if nav_result is None:
                result.message = 'Nav2 lost the goal (server stopped or restarted)'
                self.get_logger().warning(f'Attempt {attempt}/{max_attempts}: {result.message}')
                continue
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

    def _end_as_stopping(self, goal_handle, result):
        result.message = 'Navigator shutting down'
        goal_handle.abort()
        return result

    def _forward_feedback(self, goal_handle, attempt, nav_feedback_msg):
        goal_handle.publish_feedback(MoveToLocation.Feedback(
            attempt=attempt,
            distance_remaining=nav_feedback_msg.feedback.distance_remaining))

    def _wait_for_nav2(self, goal_handle, timeout):
        """Wait until the Nav2 server is up; False on timeout or cancel."""
        deadline = time.monotonic() + timeout
        while not self._nav2.server_is_ready():
            if time.monotonic() >= deadline or goal_handle.is_cancel_requested:
                return False
            if self._stopping.is_set():
                return False
            if not self.context.ok():
                raise RuntimeError('Shutting down')
            time.sleep(0.05)
        return True

    def _sleep_unless_canceled(self, goal_handle, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not goal_handle.is_cancel_requested:
            if self._stopping.is_set():
                return
            if not self.context.ok():
                raise RuntimeError('Shutting down')
            time.sleep(0.05)

    def _wait(self, future, timeout=None, give_up=None):
        """
        Block until a future is done; needs a multi-threaded executor.

        Returns None instead of the result after timeout seconds or once
        give_up() is true; give_up is checked every PEER_CHECK_PERIOD seconds.
        """
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = None if timeout is None else time.monotonic() + timeout
        next_check = time.monotonic() + PEER_CHECK_PERIOD
        while not done.wait(0.1):
            if not self.context.ok():
                raise RuntimeError('Shutting down')
            if deadline is not None and time.monotonic() >= deadline:
                return None
            if give_up is not None and time.monotonic() >= next_check:
                if give_up():
                    return None
                next_check = time.monotonic() + PEER_CHECK_PERIOD
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


def _cancel_if_accepted(send_future):
    nav_handle = send_future.result()
    if nav_handle is not None and nav_handle.accepted:
        nav_handle.cancel_goal_async()


def main(args=None):
    """Run the node."""
    return run(NavigatorNode, args)


if __name__ == '__main__':
    sys.exit(main())
