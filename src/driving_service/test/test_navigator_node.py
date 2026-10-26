"""MoveToLocation action against a fake Nav2 navigate_to_pose server."""

import math
import os
import signal
import subprocess
import sys
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

# A domain of its own and no discovery server, so no test can ever reach the
# robot's real Nav2: the robot's Fast DDS discovery server (ROS_DISCOVERY_SERVER) connects
# participants across domain IDs, so the domain ID alone does not isolate.
TEST_DOMAIN_ID = 87
TIMEOUT = 10.0


class FakeNav2(Node):
    """navigate_to_pose server that plays back scripted outcomes, one per goal."""

    def __init__(self, action_name, outcomes):
        super().__init__('fake_nav2_' + uuid.uuid4().hex[:8])
        self._executor = None  # a fake that is never spun answers nothing, like a crashed one
        self.outcomes = list(outcomes)
        self.received = []
        self.canceled = threading.Event()
        self.released = threading.Event()
        self.feedback_distance = None
        self.accept_delay = 0.0
        self.accept_delays = []  # per goal, before falling back to accept_delay
        self._answered_late = set()  # requests that got an accept_delays entry hold
        self.goals_to_reject = 0
        self.stopped = threading.Event()
        self._server = ActionServer(
            self, NavigateToPose, action_name,
            goal_callback=self._on_goal,
            execute_callback=self._execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def spin_in_background(self):
        """Spin in an executor of its own, so stop() can end it like a process."""
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        return self

    def stop(self):
        """Vanish like a crashed Nav2: running goals never get a result."""
        if self.stopped.is_set():
            return
        # destroying the action server inside a spinning executor can kill its spin
        # thread, so end the executor first, then the node, like a dying process
        self.stopped.set()
        if self._executor is not None:
            self._executor.shutdown()
        self._server.destroy()  # destroy_node() leaves action servers alive
        self.destroy_node()

    def close(self):
        self.released.set()  # held goals must not block the teardown
        self.stop()

    def _on_goal(self, goal):
        if self.accept_delays:
            # rclpy hands goal_callback and execute the same request object
            self._answered_late.add(id(goal))
            time.sleep(self.accept_delays.pop(0))
        else:
            time.sleep(self.accept_delay)
        if self.goals_to_reject > 0:
            self.goals_to_reject -= 1
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        self.received.append(goal_handle.request.pose)
        if id(goal_handle.request) in self._answered_late:
            outcome = 'hold'
        else:
            outcome = self.outcomes.pop(0) if self.outcomes else 'succeed'
        if self.feedback_distance is not None:
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = self.feedback_distance
            goal_handle.publish_feedback(feedback)
        result = NavigateToPose.Result()
        if outcome == 'hold':
            while not goal_handle.is_cancel_requested:
                if self.stopped.is_set():
                    return result
                if self.released.wait(0.01) or not self.context.ok():
                    goal_handle.abort()
                    return result
            self.canceled.set()
            goal_handle.canceled()
            return result
        if outcome == 'slow':
            time.sleep(3.0)
            goal_handle.succeed()
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
        self._nav2_action = nav2_action
        self._stopped_nav2 = []
        self.action_name = prefix + '/moveToLocation'
        overrides = {
            'action_name': self.action_name,
            'nav2_action': nav2_action,
            'nav_retry_attempts': 3,
            'nav_retry_delay': 0.0,
            'nav2_server_timeout': 1.0,
            # right after a Nav2 restart a goal request can get lost, don't wait long for it
            'nav2_response_timeout': 2.0,
        }
        overrides.update(params)
        self.navigator = NavigatorNode(parameter_overrides=[
            Parameter(name, value=value) for name, value in overrides.items()])
        self.nav2 = FakeNav2(nav2_action, outcomes).spin_in_background() if nav2_running else None
        self.client_node = Node('test_client_' + uuid.uuid4().hex[:8])
        self.client = ActionClient(self.client_node, MoveToLocation, self.action_name)
        self.feedback = []

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.navigator)
        self.executor.add_node(self.client_node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        assert self.client.wait_for_server(timeout_sec=TIMEOUT)

    def restart_nav2(self, outcomes=()):
        """Replace Nav2 by a new server under the same name, like a respawn."""
        self.nav2.stop()
        self._stopped_nav2.append(self.nav2)
        self.nav2 = FakeNav2(self._nav2_action, outcomes).spin_in_background()

    def start_zombie_nav2(self):
        """Second server under the Nav2 name that never answers, like a crashed copy."""
        zombie = FakeNav2(self._nav2_action, ())  # not spun: answers nothing
        self._stopped_nav2.append(zombie)
        return zombie

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
        for fake in (self.nav2, *self._stopped_nav2):
            if fake is not None:
                fake.close()
        self.navigator.destroy_node()
        self.client_node.destroy_node()


def wait_for(future, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)
    return future.result()


def wait_until(condition, timeout=2.0):
    """Give late messages (feedback arrives independently of results) a moment."""
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.01)
    return condition()


@pytest.fixture(autouse=True)
def ros(monkeypatch):
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
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


@pytest.mark.parametrize('bad', [
    {'nav_retry_attempts': 0},
    {'nav2_response_timeout': 0.0},
    {'nav2_server_timeout': 0.0},
    {'nav_retry_delay': -1.0},
])
def test_invalid_navigator_params_stop_the_node_at_startup(bad):
    with pytest.raises(ValueError):
        NavigatorNode(parameter_overrides=[
            Parameter(name, value=value) for name, value in bad.items()])


@pytest.mark.parametrize('goal', [
    {'x': math.nan},
    {'yaw': -math.inf},
    {'y': math.inf},
])
def test_goal_with_a_non_finite_coordinate_is_rejected(harness, goal):
    h = harness(outcomes=['succeed'])

    goal_handle = h.send(**goal)

    assert not goal_handle.accepted
    assert h.nav2.received == []


def test_cancel_while_waiting_for_the_nav2_server_ends_as_canceled(harness):
    h = harness(nav2_running=False, nav2_server_timeout=5.0)
    goal_handle = h.send(x=1.0, y=1.0)
    time.sleep(0.3)

    started = time.monotonic()
    wait_for(goal_handle.cancel_goal_async())
    result_response = wait_for(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_CANCELED
    assert result_response.result.message == 'Canceled'
    assert time.monotonic() - started < 3.0


def test_goal_ends_when_nav2_vanishes_during_the_drive(harness):
    h = harness(outcomes=['hold'], nav2_server_timeout=1.0)
    goal_handle = h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.nav2.received) == 1)

    h.nav2.stop()
    result_response = wait_for(goal_handle.get_result_async())

    assert result_response.status == GoalStatus.STATUS_ABORTED
    assert result_response.result.message == 'Nav2Unavailable'


def test_goal_is_retried_on_a_restarted_nav2(harness):
    # requests to a freshly started Nav2 can get lost for a few seconds on a busy Pi;
    # each lost one costs an attempt, so allow some
    h = harness(outcomes=['hold'], nav_retry_attempts=6)
    goal_handle = h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.nav2.received) == 1)

    h.restart_nav2(outcomes=['succeed'])
    result = wait_for(goal_handle.get_result_async(), timeout=20.0).result

    assert (result.success, result.message) == (True, 'Reached')
    assert result.attempts >= 2


def test_goal_request_without_answer_is_retried_and_a_late_acceptance_canceled(harness):
    h = harness(outcomes=['succeed'], nav2_response_timeout=1.5)
    h.nav2.accept_delays = [2.0]  # answered too late, then holds; requests are answered in turn

    result = h.move_to(x=1.0, y=1.0)

    assert (result.success, result.message, result.attempts) == (True, 'Reached', 2)
    assert wait_until(lambda: h.nav2.canceled.is_set(), timeout=5.0)


def test_goal_on_nav2_is_kept_when_a_crashed_copy_leaves_the_graph(harness):
    h = harness(outcomes=['slow'])
    zombie = h.start_zombie_nav2()
    time.sleep(1.0)  # both copies are in the graph when the goal is sent
    goal_handle = h.send(x=1.0, y=1.0)
    wait_until(lambda: len(h.nav2.received) == 1)

    zombie.stop()  # the crashed copy finally drops out
    result = wait_for(goal_handle.get_result_async()).result

    assert (result.success, result.attempts) == (True, 1)
    assert len(h.nav2.received) == 1


def run_navigator_and_ctrl_c(accept_delay, sigint_when, results=None, outcomes=('hold',),
                             params=(), received=None, nav2_mode='running', sig=signal.SIGINT,
                             next_goal=None, sigint_ignored=False):
    """
    Run the navigator as a process, signal it during a drive; True if Nav2 saw a cancel.

    nav2_mode: 'running' (fake Nav2), 'silent' (in the graph, never answers) or
    'absent'. The MoveToLocation result response, if any arrives, is appended to
    results, the poses Nav2 got are copied to received. With next_goal (a list),
    the client sends its next goal as soon as the first result arrives, like a main
    loop driving back, and appends whether it was accepted.
    """
    prefix = '/test_' + uuid.uuid4().hex[:8]
    nav2 = None if nav2_mode == 'absent' else FakeNav2(prefix + '/navigate_to_pose', outcomes)
    if nav2 is not None:
        nav2.accept_delays = [accept_delay] if accept_delay else []
    client_node = Node('test_client_' + uuid.uuid4().hex[:8])
    client = ActionClient(client_node, MoveToLocation, prefix + '/moveToLocation')
    executor = MultiThreadedExecutor()
    if nav2_mode == 'running':
        executor.add_node(nav2)
    executor.add_node(client_node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def ignore_sigint():
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    navigator = subprocess.Popen(
        [sys.executable, '-m', 'driving_service.navigator_node', '--ros-args',
         '-p', f'action_name:={prefix}/moveToLocation',
         '-p', f'nav2_action:={prefix}/navigate_to_pose', *params],
        env={**os.environ, 'ROS_DOMAIN_ID': str(TEST_DOMAIN_ID)},
        preexec_fn=ignore_sigint if sigint_ignored else None)

    def on_result(response):
        if results is not None:
            results.append(response.result())
        if next_goal is not None:
            client.send_goal_async(MoveToLocation.Goal(x=0.0, y=0.0)).add_done_callback(
                lambda f: next_goal.append(f.result().accepted))

    try:
        assert client.wait_for_server(timeout_sec=TIMEOUT)
        # Fast DDS: a request sent right after discovery can have its response dropped
        # ("client will not receive response"), give the response path a moment to match
        time.sleep(3.0)
        send_future = client.send_goal_async(MoveToLocation.Goal(x=1.0, y=1.0))
        send_future.add_done_callback(
            lambda f: f.result().get_result_async().add_done_callback(on_result))
        sigint_when(nav2)
        assert send_future.done(), 'accept response lost (Fast DDS), the test proves nothing'

        navigator.send_signal(sig)

        canceled = nav2 is not None and wait_until(lambda: nav2.canceled.is_set(), timeout=5.0)
        assert navigator.wait(timeout=TIMEOUT) is not None
        if results is not None:
            wait_until(lambda: results, timeout=2.0)
        if next_goal is not None:
            wait_until(lambda: next_goal, timeout=2.0)
        if received is not None and nav2 is not None:
            received.extend(nav2.received)
        return canceled
    finally:
        if navigator.poll() is None:
            navigator.kill()
        if nav2 is not None:
            nav2.released.set()
        rclpy.try_shutdown()
        executor.shutdown()


def test_ctrl_c_of_the_navigator_process_cancels_the_running_nav2_goal():
    def while_driving(nav2):
        assert wait_until(lambda: len(nav2.received) == 1, timeout=TIMEOUT)
        time.sleep(0.5)  # the navigator has Nav2's goal handle

    assert run_navigator_and_ctrl_c(accept_delay=0.0, sigint_when=while_driving)


def test_ctrl_c_before_nav2_accepted_still_cancels_the_nav2_goal():
    def while_nav2_decides(nav2):
        time.sleep(1.0)  # request sent, Nav2 answers only after 2 s

    assert run_navigator_and_ctrl_c(accept_delay=2.0, sigint_when=while_nav2_decides)


def test_ctrl_c_ends_the_running_goal_with_a_result_for_the_client():
    def while_driving(nav2):
        assert wait_until(lambda: len(nav2.received) == 1, timeout=TIMEOUT)
        time.sleep(0.5)

    results = []
    run_navigator_and_ctrl_c(accept_delay=0.0, sigint_when=while_driving, results=results)

    assert [(r.status, r.result.message) for r in results] == [
        (GoalStatus.STATUS_ABORTED, 'Navigator shutting down')]


def test_ctrl_c_during_the_retry_delay_ends_the_goal_with_a_result_for_the_client():
    def while_waiting_to_retry(nav2):
        assert wait_until(lambda: len(nav2.received) == 1, timeout=TIMEOUT)
        time.sleep(0.5)  # the first attempt failed, the 30 s retry delay runs

    results = []
    received = []
    run_navigator_and_ctrl_c(
        accept_delay=0.0, sigint_when=while_waiting_to_retry, results=results,
        outcomes=('abort', 'hold'), params=('-p', 'nav_retry_delay:=30.0'), received=received)

    assert [(r.status, r.result.message) for r in results] == [
        (GoalStatus.STATUS_ABORTED, 'Navigator shutting down')]
    assert len(received) == 1  # no new Nav2 goal once Ctrl-C was pressed


def test_ctrl_c_during_the_last_attempt_ends_the_goal_as_shutting_down():
    def while_driving(nav2):
        assert wait_until(lambda: len(nav2.received) == 1, timeout=TIMEOUT)
        time.sleep(0.5)

    results = []
    run_navigator_and_ctrl_c(
        accept_delay=0.0, sigint_when=while_driving, results=results,
        params=('-p', 'nav_retry_attempts:=1'))

    assert [(r.status, r.result.message) for r in results] == [
        (GoalStatus.STATUS_ABORTED, 'Navigator shutting down')]


def test_ctrl_c_while_waiting_for_an_absent_nav2_ends_the_goal_as_shutting_down():
    results = []
    run_navigator_and_ctrl_c(
        accept_delay=0.0, sigint_when=lambda _nav2: time.sleep(1.5), results=results,
        nav2_mode='absent', params=('-p', 'nav2_server_timeout:=30.0'))

    assert [(r.status, r.result.message) for r in results] == [
        (GoalStatus.STATUS_ABORTED, 'Navigator shutting down')]


def test_ctrl_c_while_waiting_for_nav2_to_answer_ends_the_goal_as_shutting_down():
    results = []
    run_navigator_and_ctrl_c(
        accept_delay=0.0, sigint_when=lambda _nav2: time.sleep(1.5), results=results,
        nav2_mode='silent',
        # a single attempt: no later loop round may be what ends the goal
        params=('-p', 'nav2_response_timeout:=30.0', '-p', 'nav_retry_attempts:=1'))

    assert [(r.status, r.result.message) for r in results] == [
        (GoalStatus.STATUS_ABORTED, 'Navigator shutting down')]


def test_goal_sent_during_shutdown_is_rejected_and_never_reaches_nav2():
    def while_driving(nav2):
        assert wait_until(lambda: len(nav2.received) == 1, timeout=TIMEOUT)
        time.sleep(0.5)

    next_goal = []
    received = []
    run_navigator_and_ctrl_c(
        accept_delay=0.0, sigint_when=while_driving, next_goal=next_goal,
        outcomes=('hold', 'hold'), received=received)

    assert next_goal in ([False], [])  # rejected, or nobody answered any more
    assert len(received) == 1


def test_sigterm_during_a_drive_cancels_the_nav2_goal_like_ctrl_c():
    def while_driving(nav2):
        assert wait_until(lambda: len(nav2.received) == 1, timeout=TIMEOUT)
        time.sleep(0.5)

    results = []
    canceled = run_navigator_and_ctrl_c(
        accept_delay=0.0, sigint_when=while_driving, results=results, sig=signal.SIGTERM)

    assert canceled
    assert [(r.status, r.result.message) for r in results] == [
        (GoalStatus.STATUS_ABORTED, 'Navigator shutting down')]


def test_ctrl_c_works_even_when_the_navigator_was_started_with_sigint_ignored():
    def while_driving(nav2):
        assert wait_until(lambda: len(nav2.received) == 1, timeout=TIMEOUT)
        time.sleep(0.5)

    assert run_navigator_and_ctrl_c(
        accept_delay=0.0, sigint_when=while_driving, sigint_ignored=True)
