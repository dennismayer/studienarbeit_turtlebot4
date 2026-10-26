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
    cancel_if_accepted, server_gone, server_identity)
from driving_service.params import check, declare_read_only, value
from driving_service.run_node import run
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


# seconds Ctrl-C waits for Nav2 to end the running goal it was told to cancel
STOP_CANCEL_TIMEOUT = 2.0
# longest Ctrl-C waits for Nav2 to answer a goal request that is still pending
# (ros2 launch kills after 10 s)
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
        self._stopping = threading.Event()  # Ctrl-C: end the running goal, start no new attempt
        self._stop_deadline = math.inf  # when Ctrl-C stops waiting for Nav2's answer

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
        End the running goal and its Nav2 goal, so Nav2 doesn't drive on without us.

        The running MoveToLocation goal ends with "Navigator shutting down" (or
        "Canceled" when the client canceled it first). A goal request Nav2 hasn't
        answered yet is waited for, at most STOP_PENDING_MAX, so a late acceptance
        is canceled, not left driving.
        """
        # before _stopping: _drive reads the deadline once it sees _stopping
        self._stop_deadline = time.monotonic() + STOP_PENDING_MAX
        self._stopping.set()
        # let _drive see it through, and rclpy send the result, before ROS goes down
        deadline = self._stop_deadline + STOP_CANCEL_TIMEOUT
        while self._busy and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        flush_until = time.monotonic() + STOP_RESULT_FLUSH
        while time.monotonic() < flush_until:
            executor.spin_once(timeout_sec=0.05)

    def _on_cancel(self, _goal_handle):
        # _drive forwards the cancel to the Nav2 goal
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
                self._sleep_unless_interrupted(goal_handle, retry_delay)
            if self._interrupted(goal_handle):
                break
            if not self._wait_for_nav2(goal_handle, server_timeout):
                if not self._interrupted(goal_handle):
                    self.get_logger().error('Nav2 action server is not available')
                    result.message = 'Nav2Unavailable'
                break
            nav2 = server_identity(self, self._nav2_name)
            result.attempts = attempt
            nav_goal = NavigateToPose.Goal(pose=self._map_pose(request.x, request.y, request.yaw))
            send_future = self._nav2.send_goal_async(
                nav_goal,
                feedback_callback=lambda msg, a=attempt: self._forward_feedback(
                    goal_handle, a, msg))
            # Ctrl-C doesn't end this wait right away: an acceptance must be canceled
            nav_handle = self._wait(
                send_future, timeout=response_timeout, give_up=self._stop_deadline_passed)
            if nav_handle is None:
                # the request or its answer got lost; never leave a late acceptance driving
                send_future.add_done_callback(lambda f: cancel_if_accepted(f.result()))
                result.message = 'Nav2 did not answer the goal request'
                self.get_logger().warning(f'Attempt {attempt}/{max_attempts}: {result.message}')
                continue
            if not nav_handle.accepted:
                result.message = 'Nav2 rejected the goal'
                self.get_logger().warning(f'Attempt {attempt}/{max_attempts}: {result.message}')
                continue
            nav_result = self._wait_for_result(goal_handle, nav_handle, nav2)
            if self._interrupted(goal_handle):
                break
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
        return self._end_unreached(goal_handle, result)

    def _interrupted(self, goal_handle):
        return goal_handle.is_cancel_requested or self._stopping.is_set()

    def _stop_deadline_passed(self):
        return self._stopping.is_set() and time.monotonic() >= self._stop_deadline

    def _end_unreached(self, goal_handle, result):
        """End a goal that didn't reach the pose; a cancel wins over Ctrl-C."""
        if goal_handle.is_cancel_requested:
            result.message = 'Canceled'
            goal_handle.canceled()
        elif self._stopping.is_set():
            result.message = 'Navigator shutting down'
            goal_handle.abort()
        else:
            goal_handle.abort()
        return result

    def _wait_for_result(self, goal_handle, nav_handle, nav2):
        """
        Wait for the result of an accepted Nav2 goal; None once its Nav2 server is gone.

        A cancel or Ctrl-C, also one that came before Nav2 accepted, cancels the Nav2 goal.
        """
        forwarded = False

        def forward_interruption_or_give_up():
            nonlocal forwarded
            if not forwarded and self._interrupted(goal_handle):
                self.get_logger().info('Canceling the Nav2 goal')
                nav_handle.cancel_goal_async()
                forwarded = True
            return server_gone(self, self._nav2_name, nav2)

        return self._wait(nav_handle.get_result_async(), give_up=forward_interruption_or_give_up)

    def _forward_feedback(self, goal_handle, attempt, nav_feedback_msg):
        goal_handle.publish_feedback(MoveToLocation.Feedback(
            attempt=attempt,
            distance_remaining=nav_feedback_msg.feedback.distance_remaining))

    def _wait_for_nav2(self, goal_handle, timeout):
        """Wait until the Nav2 server is up; False on timeout, cancel or Ctrl-C."""
        deadline = time.monotonic() + timeout
        while not self._nav2.server_is_ready():
            if time.monotonic() >= deadline or self._interrupted(goal_handle):
                return False
            self._raise_if_shut_down()
            time.sleep(0.05)
        return True

    def _sleep_unless_interrupted(self, goal_handle, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not self._interrupted(goal_handle):
            self._raise_if_shut_down()
            time.sleep(0.05)

    def _wait(self, future, timeout=None, give_up=None):
        """
        Block until a future is done; needs a multi-threaded executor.

        Returns None instead of the result after timeout seconds or once
        give_up() is true; give_up is checked every 0.1 s.
        """
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = None if timeout is None else time.monotonic() + timeout
        while not done.wait(0.1):
            self._raise_if_shut_down()
            if deadline is not None and time.monotonic() >= deadline:
                return None
            if give_up is not None and give_up():
                return None
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
