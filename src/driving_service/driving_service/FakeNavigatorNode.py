"""
Fake MoveToLocation server for tests on a robot that must not move.

Answers like the Navigator, but instead of driving it waits drive_duration
seconds and reports the goal as reached.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

import math
import sys
import threading
import time

from driving_interfaces.action import MoveToLocation
from driving_service.params import check, declare_read_only, value
from driving_service.run_node import run
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

# seconds between feedback messages and checks for a cancel or Ctrl-C
STEP = 0.1
FEEDBACK_PERIOD = 1.0
# meters per second the fake "drives", only for distance_remaining in the feedback
FAKE_SPEED = 0.3
# longest Ctrl-C waits for the running goal to end, then the rest of its result is flushed
STOP_TIMEOUT = 2.0
STOP_RESULT_FLUSH = 0.5


class FakeNavigatorNode(Node):
    """MoveToLocation server that waits drive_duration seconds instead of driving."""

    def __init__(self, **kwargs):
        super().__init__('FakeNavigator', **kwargs)
        declare_read_only(self, [
            ('action_name', 'moveToLocation', 'Name of the MoveToLocation action server'),
            ('drive_duration', 5.0, 'Seconds a goal "drives" before it is reported as reached'),
        ])
        self._drive_duration = value(self, 'drive_duration')
        check([(self._drive_duration >= 0.0, 'drive_duration must be >= 0')])
        self._busy = False
        self._busy_lock = threading.Lock()
        self._stopping = threading.Event()
        self._server = ActionServer(
            self, MoveToLocation, value(self, 'action_name'),
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().warning(
            f'Fake navigator started: goals are reported as reached after '
            f'{self._drive_duration:.1f} s, the robot does not move')

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
        """End the running goal as "Navigator shutting down", so the client gets a result."""
        self._stopping.set()
        deadline = time.monotonic() + STOP_TIMEOUT
        while self._busy and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=STEP)
        flush_until = time.monotonic() + STOP_RESULT_FLUSH
        while time.monotonic() < flush_until:
            executor.spin_once(timeout_sec=0.05)

    def destroy_node(self):
        """Destroy the action server as well, rclpy leaves it alive with the node gone."""
        self._server.destroy()
        super().destroy_node()

    def _execute(self, goal_handle):
        try:
            return self._fake_drive(goal_handle)
        finally:
            with self._busy_lock:
                self._busy = False

    def _fake_drive(self, goal_handle):
        request = goal_handle.request
        self.get_logger().info(
            f'Fake drive to x={request.x:.2f}, y={request.y:.2f}, yaw={request.yaw:.2f}')
        start = time.monotonic()
        next_feedback = start
        while True:
            elapsed = time.monotonic() - start
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Fake drive canceled')
                return MoveToLocation.Result(success=False, message='Canceled', attempts=1)
            if self._stopping.is_set():
                goal_handle.abort()
                return MoveToLocation.Result(
                    success=False, message='Navigator shutting down', attempts=1)
            if elapsed >= self._drive_duration:
                break
            if time.monotonic() >= next_feedback:
                remaining = (self._drive_duration - elapsed) * FAKE_SPEED
                goal_handle.publish_feedback(
                    MoveToLocation.Feedback(attempt=1, distance_remaining=remaining))
                next_feedback += FEEDBACK_PERIOD
            time.sleep(STEP)
        goal_handle.succeed()
        self.get_logger().info('Fake drive reached the goal')
        return MoveToLocation.Result(success=True, message='Reached', attempts=1)


def main(args=None):
    """Run the node."""
    return run(FakeNavigatorNode, args)


if __name__ == '__main__':
    sys.exit(main())
