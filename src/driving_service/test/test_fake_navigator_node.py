"""Fake navigator: MoveToLocation without driving, for tests on a robot that must not move."""

import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from driving_interfaces.action import MoveToLocation
from driving_service.FakeNavigatorNode import FakeNavigatorNode
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ros_test_tools import wait_for, wait_until

pytestmark = pytest.mark.usefixtures('ros')
DRIVE_DURATION = 0.5


class Harness:
    """Fake navigator spun like run_node's main loop, and a MoveToLocation client."""

    def __init__(self, drive_duration=DRIVE_DURATION):
        action_name = '/test_' + uuid.uuid4().hex[:8] + '/moveToLocation'
        self.navigator = FakeNavigatorNode(parameter_overrides=[
            Parameter('action_name', value=action_name),
            Parameter('drive_duration', value=drive_duration),
        ])
        self.client_node = Node('test_client_' + uuid.uuid4().hex[:8])
        self.client = ActionClient(self.client_node, MoveToLocation, action_name)
        self.feedback = []
        self.ctrl_c = threading.Event()
        self._closing = threading.Event()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.client_node)
        self.navigator_executor = MultiThreadedExecutor()
        self.navigator_executor.add_node(self.navigator)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        self._navigator_thread = threading.Thread(target=self._run_navigator, daemon=True)
        self._navigator_thread.start()
        assert self.client.wait_for_server(timeout_sec=5.0)

    def _run_navigator(self):
        # like run_node.run: spin until Ctrl-C, then let the node wind down
        while not self.ctrl_c.is_set() and not self._closing.is_set():
            self.navigator_executor.spin_once(timeout_sec=0.05)
        if self.ctrl_c.is_set():
            self.navigator.stop(self.navigator_executor)

    def send(self, x=1.0, y=2.0, yaw=0.5):
        goal = MoveToLocation.Goal(x=x, y=y, yaw=yaw)
        return wait_for(self.client.send_goal_async(
            goal, feedback_callback=lambda msg: self.feedback.append(msg.feedback)))

    def close(self):
        # destroy everything while ROS is still up; leftovers collected later can segfault
        self._closing.set()
        self._navigator_thread.join(timeout=5.0)
        self.executor.shutdown()
        self.navigator_executor.shutdown()
        self.navigator.destroy_node()
        self.client.destroy()
        self.client_node.destroy_node()
        rclpy.try_shutdown()


@pytest.fixture
def harness():
    created = []

    def make(**kwargs):
        created.append(Harness(**kwargs))
        return created[-1]

    yield make
    for h in created:
        h.close()


def test_goal_succeeds_after_the_drive_duration(harness):
    h = harness()
    start = time.monotonic()

    goal_handle = h.send()
    response = wait_for(goal_handle.get_result_async())

    assert goal_handle.accepted
    assert time.monotonic() - start >= DRIVE_DURATION
    assert response.status == GoalStatus.STATUS_SUCCEEDED
    assert response.result.success
    assert response.result.message == 'Reached'
    assert response.result.attempts == 1


def test_feedback_reports_the_first_attempt(harness):
    h = harness(drive_duration=1.5)

    goal_handle = h.send()
    wait_for(goal_handle.get_result_async())

    assert h.feedback
    assert all(feedback.attempt == 1 for feedback in h.feedback)


def test_goal_while_driving_is_rejected(harness):
    h = harness(drive_duration=2.0)
    first = h.send()

    second = h.send()

    assert first.accepted
    assert not second.accepted


def test_next_goal_is_accepted_after_the_last_one_ended(harness):
    h = harness()
    wait_for(h.send().get_result_async())

    assert h.send().accepted


def test_goal_with_a_non_finite_coordinate_is_rejected(harness):
    h = harness()

    assert not h.send(x=float('nan')).accepted


def test_cancel_ends_the_goal_as_canceled_before_the_drive_duration(harness):
    h = harness(drive_duration=5.0)
    goal_handle = h.send()
    start = time.monotonic()

    wait_for(goal_handle.cancel_goal_async())
    response = wait_for(goal_handle.get_result_async())

    assert time.monotonic() - start < 2.0
    assert response.status == GoalStatus.STATUS_CANCELED
    assert not response.result.success
    assert response.result.message == 'Canceled'


def test_ctrl_c_ends_the_running_goal_so_the_client_gets_a_result(harness):
    h = harness(drive_duration=5.0)
    goal_handle = h.send()
    result = goal_handle.get_result_async()

    h.ctrl_c.set()

    assert wait_until(result.done, timeout=3.0)
    assert result.result().status == GoalStatus.STATUS_ABORTED
    assert result.result().result.message == 'Navigator shutting down'
