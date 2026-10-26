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
from driving_service.action_peer import (
    cancel_if_accepted, PEER_CHECK_PERIOD, PendingRequest, server_gone, server_identity)
from driving_service.params import check, declare_read_only, value
from driving_service.run_node import run
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


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
        declare_read_only(self, [
            ('action_name', 'moveToLocation', 'Name of the MoveToLocation action server'),
            ('nav2_action', 'navigate_to_pose', "Name of Nav2's NavigateToPose action server"),
            ('nav_retry_attempts', 3, 'Nav2 attempts per goal, including the first one'),
            ('nav_retry_delay', 5.0, 'Seconds to wait before retrying a failed Nav2 goal'),
            ('nav2_server_timeout', 5.0, 'Seconds to wait for the Nav2 action server per goal'),
            ('nav2_response_timeout', 5.0, 'Seconds to wait for Nav2 to accept or reject a goal'),
        ])
        check([
            (value(self, 'nav_retry_attempts') >= 1, 'nav_retry_attempts must be >= 1'),
            (value(self, 'nav_retry_delay') >= 0.0, 'nav_retry_delay must be >= 0'),
            (value(self, 'nav2_server_timeout') > 0.0, 'nav2_server_timeout must be > 0'),
            (value(self, 'nav2_response_timeout') > 0.0, 'nav2_response_timeout must be > 0'),
        ])
        self._busy = False
        self._busy_lock = threading.Lock()
        self._nav_handle = None
        self._pending = None  # PendingRequest to Nav2, deadline in monotonic time
        self._stopping = threading.Event()  # Ctrl-C: end the running goal, start no new attempt

        # reentrant, so Nav2 responses arrive while a goal is being executed
        group = ReentrantCallbackGroup()
        self._nav2_name = value(self, 'nav2_action')
        self._nav2 = ActionClient(
            self, NavigateToPose, self._nav2_name,
            callback_group=group)
        self._server = ActionServer(
            self, MoveToLocation, value(self, 'action_name'),
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._execute,
            callback_group=group)

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
        pending = self._pending
        self._stopping.set()
        if pending is not None:
            # Nav2 may still accept the goal we just sent; wait for its answer as long as
            # the response timeout would have (at most STOP_PENDING_MAX, ros2 launch kills
            # after 10 s), so a late acceptance is canceled, not orphaned
            remaining = pending.deadline - time.monotonic()
            executor.spin_until_future_complete(
                pending.future, timeout_sec=min(max(0.0, remaining), STOP_PENDING_MAX))
            if pending.future.done() and pending.future.result().accepted:
                self._nav_handle = pending.future.result()
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
        server_timeout = value(self, 'nav2_server_timeout')
        retry_delay = value(self, 'nav_retry_delay')
        response_timeout = value(self, 'nav2_response_timeout')
        max_attempts = value(self, 'nav_retry_attempts')
        result = MoveToLocation.Result()
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                self._sleep_unless_canceled(goal_handle, retry_delay)
                if goal_handle.is_cancel_requested:
                    return self._end_as_canceled(goal_handle, result)
                if self._stopping.is_set():
                    return self._end_as_stopping(goal_handle, result)
            if not self._wait_for_nav2(goal_handle, server_timeout):
                if self._stopping.is_set():
                    return self._end_as_stopping(goal_handle, result)
                if goal_handle.is_cancel_requested:
                    return self._end_as_canceled(goal_handle, result)
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
            self._pending = PendingRequest(send_future, time.monotonic() + response_timeout)
            nav_handle = self._wait(
                send_future, timeout=response_timeout, give_up=self._stopping.is_set)
            self._pending = None
            if nav_handle is None:
                # the request or its answer got lost; never leave a late acceptance driving
                send_future.add_done_callback(lambda f: cancel_if_accepted(f.result()))
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
                return self._end_as_canceled(goal_handle, result)
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

    def _end_as_canceled(self, goal_handle, result):
        result.message = 'Canceled'
        goal_handle.canceled()
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
            self._raise_if_shut_down()
            time.sleep(0.05)
        return True

    def _sleep_unless_canceled(self, goal_handle, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not goal_handle.is_cancel_requested:
            if self._stopping.is_set():
                return
            self._raise_if_shut_down()
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
            self._raise_if_shut_down()
            if deadline is not None and time.monotonic() >= deadline:
                return None
            if give_up is not None and time.monotonic() >= next_check:
                if give_up():
                    return None
                next_check = time.monotonic() + PEER_CHECK_PERIOD
        return future.result()

    def _raise_if_shut_down(self):
        if not self.context.ok():
            raise RuntimeError('Shutting down')

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
