"""MoveToLocation action against a fake Nav2 navigate_to_pose server."""

import math
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from driving_interfaces.action import MoveToLocation
from driving_service.navigator_node import NavigatorNode
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

# a domain of its own, so no test can ever reach the robot's real Nav2
TEST_DOMAIN_ID = 87
TIMEOUT = 10.0


class FakeNav2(Node):
    """navigate_to_pose server that plays back scripted outcomes, one per goal."""

    def __init__(self, action_name, outcomes):
        super().__init__('fake_nav2_' + uuid.uuid4().hex[:8])
        self.outcomes = list(outcomes)
        self.received = []
        self.canceled = threading.Event()
        self.released = threading.Event()
        self.feedback_distance = None
        self.accept_delay = 0.0
        self.goals_to_reject = 0
        self._server = ActionServer(
            self, NavigateToPose, action_name,
            goal_callback=self._on_goal,
            execute_callback=self._execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def _on_goal(self, _goal):
        time.sleep(self.accept_delay)
        if self.goals_to_reject > 0:
            self.goals_to_reject -= 1
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        self.received.append(goal_handle.request.pose)
        outcome = self.outcomes.pop(0) if self.outcomes else 'succeed'
        if self.feedback_distance is not None:
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = self.feedback_distance
            goal_handle.publish_feedback(feedback)
        result = NavigateToPose.Result()
        if outcome == 'hold':
            while not goal_handle.is_cancel_requested:
                if self.released.wait(0.01) or not self.context.ok():
                    goal_handle.abort()
                    return result
            self.canceled.set()
            goal_handle.canceled()
            return result
        if outcome == 'abort':
            result.error_code = 104
            result.error_msg = 'no valid path found'
            goal_handle.abort()
            return result
        goal_handle.succeed()
        return result


class Harness:
    """Navigator, fake Nav2 and a MoveToLocation client spinning in one executor."""

    def __init__(self, outcomes=(), nav2_running=True, **params):
        prefix = '/test_' + uuid.uuid4().hex[:8]
        nav2_action = prefix + '/navigate_to_pose'
        self.action_name = prefix + '/moveToLocation'
        overrides = {
            'action_name': self.action_name,
            'nav2_action': nav2_action,
            'nav_retry_attempts': 3,
            'nav_retry_delay': 0.0,
            'nav2_server_timeout': 1.0,
        }
        overrides.update(params)
        self.navigator = NavigatorNode(parameter_overrides=[
            Parameter(name, value=value) for name, value in overrides.items()])
        self.nav2 = FakeNav2(nav2_action, outcomes) if nav2_running else None
        self.client_node = Node('test_client_' + uuid.uuid4().hex[:8])
        self.client = ActionClient(self.client_node, MoveToLocation, self.action_name)
        self.feedback = []

        self.executor = MultiThreadedExecutor()
        for node in (self.navigator, self.nav2, self.client_node):
            if node is not None:
                self.executor.add_node(node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        assert self.client.wait_for_server(timeout_sec=TIMEOUT)

    def send(self, x=0.0, y=0.0, yaw=0.0):
        """Send a goal, return its goal handle once accepted or rejected."""
        goal = MoveToLocation.Goal(x=x, y=y, yaw=yaw)
        future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        return wait_for(future)

    def move_to(self, **goal):
        """Send a goal and return its result."""
        goal_handle = self.send(**goal)
        assert goal_handle.accepted
        return wait_for(goal_handle.get_result_async()).result

    def _on_feedback(self, msg):
        self.feedback.append(msg.feedback)

    def close(self):
        if self.nav2 is not None:
            self.nav2.released.set()  # held goals must not block the teardown
        rclpy.try_shutdown()  # lets a navigator that waits on Nav2 give up
        self.executor.shutdown()
        for node in (self.navigator, self.nav2, self.client_node):
            if node is not None:
                node.destroy_node()


def wait_for(future):
    deadline = time.monotonic() + TIMEOUT
    while not future.done():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)
    return future.result()


def wait_until(condition, timeout=2.0):
    """Give late messages (feedback arrives independently of results) a moment."""
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.01)


@pytest.fixture(autouse=True)
def ros():
    rclpy.init(domain_id=TEST_DOMAIN_ID)
    yield
    rclpy.try_shutdown()


@pytest.fixture
def harness():
    created = []

    def make(*args, **kwargs):
        created.append(Harness(*args, **kwargs))
        return created[-1]

    yield make
    for h in created:
        h.close()


def test_goal_is_reached_and_nav2_gets_the_pose_in_the_map_frame(harness):
    h = harness(outcomes=['succeed'])

    result = h.move_to(x=1.5, y=-2.0, yaw=math.pi / 2)

    assert (result.success, result.message, result.attempts) == (True, 'Reached', 1)
    pose = h.nav2.received[0]
    assert pose.header.frame_id == 'map'
    assert (pose.pose.position.x, pose.pose.position.y) == (1.5, -2.0)
    assert pose.pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert pose.pose.orientation.w == pytest.approx(math.cos(math.pi / 4))


def test_failed_nav2_goal_is_retried_until_reached(harness):
    h = harness(outcomes=['abort', 'succeed'])

    result = h.move_to(x=1.0, y=1.0)

    assert (result.success, result.message, result.attempts) == (True, 'Reached', 2)
    assert len(h.nav2.received) == 2


def test_goal_fails_with_nav2_error_after_all_attempts(harness):
    h = harness(outcomes=['abort', 'abort', 'abort'], nav_retry_attempts=3)

    goal_handle = h.send(x=1.0, y=1.0)
    result_response = wait_for(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_ABORTED
    result = result_response.result
    assert (result.success, result.attempts) == (False, 3)
    assert 'no valid path found' in result.message
    assert len(h.nav2.received) == 3


def test_nav2_distance_remaining_is_forwarded_with_the_attempt(harness):
    h = harness(outcomes=['abort', 'succeed'])
    h.nav2.feedback_distance = 4.25

    h.move_to(x=1.0, y=1.0)

    wait_until(lambda: len(h.feedback) >= 2)
    assert [(f.attempt, f.distance_remaining) for f in h.feedback] == [(1, 4.25), (2, 4.25)]


def test_second_goal_is_rejected_while_driving(harness):
    h = harness(outcomes=['hold'])
    first = h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.nav2.received) == 1)

    second = h.send(x=5.0, y=5.0)

    assert first.accepted
    assert not second.accepted
    wait_for(first.cancel_goal_async())


def test_canceling_the_goal_cancels_the_nav2_goal(harness):
    h = harness(outcomes=['hold'])
    h.nav2.feedback_distance = 3.0
    goal_handle = h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.feedback) == 1)
    time.sleep(0.3)  # Nav2's goal handle has reached the navigator

    wait_for(goal_handle.cancel_goal_async())
    result_response = wait_for(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_CANCELED
    assert (result_response.result.success, result_response.result.message) == (False, 'Canceled')
    assert h.nav2.canceled.is_set()
    assert len(h.nav2.received) == 1


def test_cancel_that_arrives_before_nav2_accepted_still_cancels_the_nav2_goal(harness):
    h = harness(outcomes=['hold'])
    h.nav2.accept_delay = 1.0
    goal_handle = h.send(x=1.0, y=1.0)
    time.sleep(0.3)  # navigator is waiting for Nav2 to accept

    wait_for(goal_handle.cancel_goal_async())
    result_response = wait_for(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_CANCELED
    assert h.nav2.canceled.is_set()


def test_retry_waits_the_retry_delay(harness):
    h = harness(outcomes=['abort', 'succeed'], nav_retry_delay=1.0)

    started = time.monotonic()
    result = h.move_to(x=1.0, y=1.0)

    assert result.success
    assert time.monotonic() - started >= 1.0


def test_cancel_during_the_retry_delay_ends_the_goal_without_another_attempt(harness):
    h = harness(outcomes=['abort', 'succeed'], nav_retry_delay=30.0)
    goal_handle = h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.nav2.received) == 1)
    time.sleep(0.3)  # first attempt has failed, the delay is running

    started = time.monotonic()
    wait_for(goal_handle.cancel_goal_async())
    result_response = wait_for(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_CANCELED
    assert time.monotonic() - started < 10.0  # far below the 30 s delay
    assert len(h.nav2.received) == 1


def test_goal_fails_right_away_when_nav2_is_not_running(harness):
    h = harness(nav2_running=False, nav2_server_timeout=0.5)

    goal_handle = h.send(x=1.0, y=1.0)
    result_response = wait_for(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_ABORTED
    result = result_response.result
    assert (result.success, result.message, result.attempts) == (False, 'Nav2Unavailable', 0)


def test_goal_rejected_by_nav2_counts_as_a_failed_attempt(harness):
    h = harness(outcomes=['succeed'])
    h.nav2.goals_to_reject = 1

    result = h.move_to(x=1.0, y=1.0)

    assert (result.success, result.message, result.attempts) == (True, 'Reached', 2)


def test_goal_rejected_by_nav2_every_time_fails_with_a_rejection_message(harness):
    h = harness(nav_retry_attempts=2)
    h.nav2.goals_to_reject = 2

    goal_handle = h.send(x=1.0, y=1.0)
    result = wait_for(goal_handle.get_result_async()).result

    assert (result.success, result.attempts) == (False, 2)
    assert result.message == 'Nav2 rejected the goal'


def test_next_goal_is_accepted_after_the_previous_one_finished(harness):
    h = harness(outcomes=['succeed', 'succeed'])

    first = h.move_to(x=1.0, y=1.0)
    second = h.move_to(x=2.0, y=2.0)

    assert (first.success, second.success) == (True, True)


def shuts_down_within(h, seconds):
    """Shut ROS down like Ctrl-C does and report whether the executor stopped in time."""
    def shutdown():
        rclpy.try_shutdown()
        h.executor.shutdown()

    thread = threading.Thread(target=shutdown, daemon=True)
    thread.start()
    thread.join(seconds)
    return not thread.is_alive()


def test_shutdown_while_driving_does_not_hang(harness):
    h = harness(outcomes=['hold'])
    h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.nav2.received) == 1)

    assert shuts_down_within(h, 5.0)


def test_shutdown_while_waiting_to_retry_does_not_hang(harness):
    h = harness(outcomes=['abort'], nav_retry_delay=30.0)
    h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.nav2.received) == 1)
    time.sleep(0.3)

    assert shuts_down_within(h, 5.0)


def test_cancel_during_the_last_attempt_ends_as_canceled(harness):
    h = harness(outcomes=['hold'], nav_retry_attempts=1)
    h.nav2.feedback_distance = 3.0
    goal_handle = h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.feedback) == 1)
    time.sleep(0.3)

    wait_for(goal_handle.cancel_goal_async())
    result_response = wait_for(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_CANCELED
    assert result_response.result.message == 'Canceled'
