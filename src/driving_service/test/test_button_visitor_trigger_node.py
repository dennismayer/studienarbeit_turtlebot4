"""Button visitor trigger: a TurtleBot4 button press starts a visit instead of the LiDAR."""

import threading
import time
import uuid

from driving_interfaces.action import VisitorDetected
from driving_service.ButtonVisitorTriggerNode import ButtonVisitorTriggerNode
from irobot_create_msgs.msg import InterfaceButtons
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_test_tools import wait_until

pytestmark = pytest.mark.usefixtures('ros')


class FakeReceptionist(Node):
    """VisitorDetected server: records goals, ends each visit right away."""

    def __init__(self, action_name, accept=True):
        super().__init__('fake_receptionist_' + uuid.uuid4().hex[:8])
        self.goals = []
        self.accept = accept
        self._server = ActionServer(
            self, VisitorDetected, action_name,
            goal_callback=lambda _: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            execute_callback=self._execute,
            callback_group=ReentrantCallbackGroup())

    def close(self):
        self._server.destroy()  # destroy_node() leaves action servers alive
        self.destroy_node()

    def _execute(self, goal_handle):
        self.goals.append(goal_handle.request)
        goal_handle.succeed()
        return VisitorDetected.Result(success=True, message='Visit over')


class Harness:
    """Trigger node, fake receptionist and a button publisher in one executor."""

    def __init__(self, **params):
        prefix = '/test_' + uuid.uuid4().hex[:8]
        overrides = {
            'action_name': prefix + '/visitorDetected',
            'buttons_topic': prefix + '/interface_buttons',
        }
        overrides.update(params)
        self.trigger = ButtonVisitorTriggerNode(parameter_overrides=[
            Parameter(name, value=value) for name, value in overrides.items()])
        self.receptionist = FakeReceptionist(overrides['action_name'])
        self.publisher_node = Node('test_buttons_' + uuid.uuid4().hex[:8])
        self.publisher = self.publisher_node.create_publisher(
            InterfaceButtons, overrides['buttons_topic'], qos_profile_sensor_data)
        # tells when the receptionist is reachable under the action name
        self._readiness_probe = ActionClient(
            self.publisher_node, VisitorDetected, overrides['action_name'])
        self.executor = MultiThreadedExecutor()
        for node in (self.trigger, self.receptionist, self.publisher_node):
            self.executor.add_node(node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        assert wait_until(lambda: self.publisher.get_subscription_count() > 0)
        assert wait_until(self._readiness_probe.server_is_ready)

    def buttons(self, button_1=False, button_2=False, button_power=False):
        msg = InterfaceButtons()
        msg.button_1.is_pressed = button_1
        msg.button_2.is_pressed = button_2
        msg.button_power.is_pressed = button_power
        self.publisher.publish(msg)
        time.sleep(0.05)

    def close(self):
        # destroy everything while ROS is still up; leftovers collected later can segfault
        self.executor.shutdown()
        self.trigger.destroy_node()
        self.receptionist.close()
        self._readiness_probe.destroy()
        self.publisher_node.destroy_node()
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


def test_press_starts_one_visit_with_the_configured_visitor(harness):
    h = harness(visitor_distance=1.3, visitor_angle=0.2, visitor_confidence=0.8)

    h.buttons(button_1=True)

    assert wait_until(lambda: len(h.receptionist.goals) == 1)
    goal = h.receptionist.goals[0]
    assert goal.distance == pytest.approx(1.3)
    assert goal.angle == pytest.approx(0.2)
    assert goal.confidence == pytest.approx(0.8)


def test_holding_the_button_starts_only_one_visit(harness):
    h = harness()

    for _ in range(5):
        h.buttons(button_1=True)

    assert wait_until(lambda: len(h.receptionist.goals) == 1)
    assert not wait_until(lambda: len(h.receptionist.goals) > 1, timeout=0.5)


def test_release_and_press_again_starts_the_next_visit(harness):
    h = harness()
    h.buttons(button_1=True)
    assert wait_until(lambda: len(h.receptionist.goals) == 1)

    h.buttons(button_1=False)
    h.buttons(button_1=True)

    assert wait_until(lambda: len(h.receptionist.goals) == 2)


def test_other_buttons_start_no_visit(harness):
    h = harness()

    h.buttons(button_2=True)
    h.buttons(button_power=True)

    assert not wait_until(lambda: h.receptionist.goals, timeout=0.5)


def test_configured_button_2_starts_a_visit(harness):
    h = harness(button='button_2')

    h.buttons(button_1=True)
    h.buttons(button_2=True)

    assert wait_until(lambda: len(h.receptionist.goals) == 1)


def test_press_while_the_receptionist_rejects_does_not_block_the_next_press(harness):
    h = harness()
    h.receptionist.accept = False
    h.buttons(button_1=True)
    h.buttons(button_1=False)
    assert not wait_until(lambda: h.receptionist.goals, timeout=0.5)

    h.receptionist.accept = True
    h.buttons(button_1=True)

    assert wait_until(lambda: len(h.receptionist.goals) == 1)
