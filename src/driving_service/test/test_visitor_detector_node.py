"""Visitor detector node: /scan in, VisitorDetected goals out to a fake receptionist."""

import math
import threading
import time
import uuid

from driving_interfaces.action import VisitorDetected
from driving_service.VisitorDetectorNode import VisitorDetectorNode
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_test_tools import SpinsAlone, wait_until
from sensor_msgs.msg import LaserScan

pytestmark = pytest.mark.usefixtures('ros')
NUM_POINTS = 720
ANGLE_MIN = -math.pi + 0.0175
ANGLE_INCREMENT = 2 * math.pi / NUM_POINTS
FRONT = -math.pi / 2


def scan(person=True):
    """Scan with a person 1.2 m straight ahead (feet together, 0.3 m), or an empty one."""
    msg = LaserScan()
    msg.header.frame_id = 'rplidar_link'
    msg.angle_min = ANGLE_MIN
    msg.angle_increment = ANGLE_INCREMENT
    msg.angle_max = ANGLE_MIN + (NUM_POINTS - 1) * ANGLE_INCREMENT
    msg.range_min = 0.15
    msg.range_max = 12.0
    ranges = [math.inf] * NUM_POINTS
    if person:
        half_span = math.atan2(0.15, 1.2)
        for i in range(NUM_POINTS):
            angle = ANGLE_MIN + i * ANGLE_INCREMENT
            offset = math.atan2(math.sin(angle - FRONT), math.cos(angle - FRONT))
            if abs(offset) <= half_span:
                ranges[i] = 1.2 / math.cos(offset)
    msg.ranges = ranges
    return msg


class FakeReceptionist(SpinsAlone, Node):
    """VisitorDetected server: records goals and keeps each visit open until told to end it."""

    def __init__(self, action_name, accept=True):
        super().__init__('fake_receptionist_' + uuid.uuid4().hex[:8])
        self.goals = []
        self.offered = 0
        self.accept = accept
        self.answer_delays = []  # per goal; a goal answered late waits to be canceled
        self.canceled = threading.Event()
        self._answered_late = set()
        self._end_visit = threading.Event()
        self.stopped = threading.Event()
        self._executor = None  # a fake that is never spun answers nothing, like a crashed one
        self._server = ActionServer(
            self, VisitorDetected, action_name,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup())

    def _on_goal(self, goal):
        self.offered += 1
        if self.answer_delays:
            # rclpy hands goal_callback and execute the same request object
            self._answered_late.add(id(goal))
            time.sleep(self.answer_delays.pop(0))
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def _execute(self, goal_handle):
        if id(goal_handle.request) in self._answered_late:
            while not goal_handle.is_cancel_requested and self.context.ok():
                time.sleep(0.01)
            self.canceled.set()
            goal_handle.canceled()
            return VisitorDetected.Result()
        self.goals.append(goal_handle.request)
        while not self._end_visit.wait(0.01):
            if self.stopped.is_set():
                return VisitorDetected.Result()
            if not self.context.ok():
                break
        self._end_visit.clear()
        goal_handle.succeed()
        return VisitorDetected.Result(success=True, message='Visit over')

    def end_visit(self):
        self._end_visit.set()


class Harness:
    """Detector node, fake receptionist and a scan publisher in one executor."""

    def __init__(self, receptionist=True, accept=True, **params):
        prefix = '/test_' + uuid.uuid4().hex[:8]
        action_name = prefix + '/visitorDetected'
        scan_topic = prefix + '/scan'
        overrides = {
            'action_name': action_name,
            'scan_topic': scan_topic,
            'stability_threshold': 8,
            'lidar_frame_skip_rate': 1,
            'visitor_timeout': 0.5,
            'cooldown_period': 0.5,
        }
        overrides.update(params)
        self.detector = VisitorDetectorNode(parameter_overrides=[
            Parameter(name, value=value) for name, value in overrides.items()])
        self.receptionist = None
        self._receptionists = []
        self._action_name = action_name
        self.publisher_node = Node('test_scan_' + uuid.uuid4().hex[:8])
        self.publisher = self.publisher_node.create_publisher(
            LaserScan, scan_topic, qos_profile_sensor_data)
        # tells when a receptionist is reachable under the action name
        self._readiness_probe = ActionClient(self.publisher_node, VisitorDetected, action_name)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.detector)
        self.executor.add_node(self.publisher_node)
        if receptionist:
            self.start_receptionist(accept)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        wait_until(lambda: self.publisher.get_subscription_count() > 0)

    def start_receptionist(self, accept=True):
        """Start the receptionist, or replace the running one like a restart."""
        if self.receptionist is not None:
            self.receptionist.stop()
        self.receptionist = FakeReceptionist(self._action_name, accept).spin_in_background()
        self._receptionists.append(self.receptionist)
        wait_until(lambda: self._readiness_probe.server_is_ready())

    def start_zombie_receptionist(self):
        """Second server under the same name that never answers, like a crashed copy."""
        zombie = FakeReceptionist(self._action_name)  # not spun: answers nothing
        self._receptionists.append(zombie)
        return zombie

    def publish(self, scans, person=True):
        """Publish scans at about 50 Hz."""
        for _ in range(scans):
            self.publisher.publish(scan(person))
            time.sleep(0.02)

    def close(self):
        if self.receptionist is not None:
            self.receptionist.end_visit()
        rclpy.try_shutdown()
        self.executor.shutdown()
        for receptionist in self._receptionists:
            receptionist.stop()


@pytest.fixture
def harness():
    created = []

    def make(*args, **kwargs):
        created.append(Harness(*args, **kwargs))
        return created[-1]

    yield make
    for h in created:
        h.close()


def test_visitor_in_front_starts_one_visit_with_its_position(harness):
    h = harness()

    h.publish(40)

    assert wait_until(lambda: len(h.receptionist.goals) >= 1)
    h.publish(40)
    assert len(h.receptionist.goals) == 1
    goal = h.receptionist.goals[0]
    assert goal.distance == pytest.approx(1.2, abs=0.05)
    assert goal.angle == pytest.approx(0.0, abs=0.03)
    assert goal.confidence >= 0.75


def test_next_visitor_starts_a_new_visit_after_the_scene_was_clear(harness):
    h = harness(visitor_timeout=0.5, cooldown_period=0.5)
    h.publish(20)
    assert wait_until(lambda: len(h.receptionist.goals) == 1)

    h.receptionist.end_visit()
    h.publish(75, person=False)  # 1.5 s: clear and cooldown over
    h.publish(30)

    assert wait_until(lambda: len(h.receptionist.goals) == 2)


def test_visitor_is_reported_once_the_receptionist_comes_up(harness):
    h = harness(receptionist=False)
    h.publish(30)  # stable detection, but nobody serves it

    h.start_receptionist()
    h.publish(30)

    assert wait_until(lambda: len(h.receptionist.goals) == 1)


def test_rejected_visitor_is_offered_again_only_after_the_scene_was_clear(harness):
    h = harness(accept=False, visitor_timeout=0.5, cooldown_period=0.5)
    h.publish(20)
    assert wait_until(lambda: h.receptionist.offered == 1)

    h.publish(100)  # visitor stays for 2 s
    offered_while_staying = h.receptionist.offered
    h.publish(75, person=False)
    h.publish(30)

    assert offered_while_staying == 1
    assert wait_until(lambda: h.receptionist.offered == 2)


def test_visitor_who_steps_away_shorter_than_the_timeout_does_not_start_a_new_visit(harness):
    h = harness(visitor_timeout=3.0, cooldown_period=0.5)
    h.publish(20)
    assert wait_until(lambda: len(h.receptionist.goals) == 1)

    h.receptionist.end_visit()
    h.publish(30, person=False)  # 0.6 s away
    h.publish(75)  # back for 1.5 s

    assert len(h.receptionist.goals) == 1


def test_detector_rearms_when_the_receptionist_restarts_during_a_visit(harness):
    h = harness(visitor_timeout=0.5, cooldown_period=0.5)
    h.publish(20)
    assert wait_until(lambda: len(h.receptionist.goals) == 1)

    h.start_receptionist()  # the old one vanishes with the visit still open
    h.publish(100, person=False)  # 2 s: lost visit noticed, clear and cooldown over
    h.publish(30)

    assert wait_until(lambda: len(h.receptionist.goals) == 1)


def test_detector_rearms_when_the_receptionist_stops_during_a_visit(harness):
    h = harness(visitor_timeout=0.5, cooldown_period=0.5)
    h.publish(20)
    assert wait_until(lambda: len(h.receptionist.goals) == 1)

    h.receptionist.stop()
    h.publish(100, person=False)  # 2 s without receptionist
    h.start_receptionist()
    h.publish(30)

    assert wait_until(lambda: len(h.receptionist.goals) == 1)


def test_goal_request_without_answer_counts_as_not_started_and_a_late_acceptance_is_canceled(
        harness):
    h = harness(receptionist_response_timeout=0.5)
    h.receptionist.answer_delays = [2.0]  # answered too late, then waits to be canceled

    h.publish(60)

    assert wait_until(lambda: len(h.receptionist.goals) == 1)
    assert wait_until(lambda: h.receptionist.canceled.is_set(), timeout=5.0)


def test_open_visit_is_kept_when_a_crashed_receptionist_copy_leaves_the_graph(harness):
    h = harness(visitor_timeout=0.5, cooldown_period=0.5)
    zombie = h.start_zombie_receptionist()
    time.sleep(1.0)  # both copies are in the graph when the visitor is sent
    h.publish(20)
    assert wait_until(lambda: len(h.receptionist.goals) == 1)

    zombie.stop()  # the crashed copy finally drops out
    h.publish(75, person=False)  # 1.5 s: a lost visit would re-arm by now
    h.publish(30)

    assert h.receptionist.offered == 1


@pytest.mark.parametrize('bad', [
    {'receptionist_response_timeout': 0.0},
    {'stability_threshold': 2},
])
def test_invalid_detector_params_stop_the_node_at_startup(bad):
    with pytest.raises(ValueError):
        VisitorDetectorNode(parameter_overrides=[
            Parameter(name, value=value) for name, value in bad.items()])
