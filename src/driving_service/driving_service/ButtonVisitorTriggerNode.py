"""
Button visitor trigger for tests: a TurtleBot4 button press starts a visit.

Replaces the visitor detector when the LiDAR detection should not decide when a
visit starts. Each press sends one VisitorDetected goal with a fixed, generic
visitor; the receptionist accepts or rejects it as it would a detected one.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

import math
import sys

from driving_interfaces.action import VisitorDetected
from driving_service.params import check, declare_read_only, value
from driving_service.run_node import run
from irobot_create_msgs.msg import InterfaceButtons
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# the faceplate buttons a press may come from; the power button is left out on purpose
TRIGGER_BUTTONS = ('button_1', 'button_2')


class ButtonVisitorTriggerNode(Node):
    """Sends a VisitorDetected goal per press of the configured button."""

    def __init__(self, **kwargs):
        super().__init__('ButtonVisitorTrigger', **kwargs)
        declare_read_only(self, [
            ('action_name', 'visitorDetected',
             'VisitorDetected action server, served by the receptionist main loop'),
            ('buttons_topic', '/interface_buttons', 'InterfaceButtons topic of the Create 3 base'),
            ('button', 'button_1', 'Faceplate button that starts a visit: button_1 or button_2'),
            ('visitor_distance', 1.0, 'Distance in meters reported for the pressed visitor'),
            ('visitor_angle', 0.0, 'Angle in radians reported for the pressed visitor'),
            ('visitor_confidence', 0.9, 'Confidence (0..1) reported for the pressed visitor'),
        ])
        self._button = value(self, 'button')
        self._goal = VisitorDetected.Goal(
            distance=value(self, 'visitor_distance'),
            angle=value(self, 'visitor_angle'),
            confidence=value(self, 'visitor_confidence'))
        check([
            (self._button in TRIGGER_BUTTONS, 'button must be button_1 or button_2'),
            (self._goal.distance > 0.0, 'visitor_distance must be > 0'),
            (0.0 <= self._goal.confidence <= 1.0, 'visitor_confidence must be within 0..1'),
        ])
        self._client = ActionClient(self, VisitorDetected, value(self, 'action_name'))
        self._was_pressed = False
        self._buttons_sub = self.create_subscription(
            InterfaceButtons, value(self, 'buttons_topic'), self._on_buttons,
            qos_profile_sensor_data)
        self.get_logger().warning(
            f'Button visitor trigger started: press {self._button} to start a visit, '
            'the LiDAR is not used')

    def destroy_node(self):
        """Destroy the action client as well, rclpy leaves it alive with the node gone."""
        self._client.destroy()
        super().destroy_node()

    def _on_buttons(self, msg: InterfaceButtons) -> None:
        pressed = getattr(msg, self._button).is_pressed
        # only the press itself counts, not the button being held down
        if pressed and not self._was_pressed:
            self._start_visit()
        self._was_pressed = pressed

    def _start_visit(self) -> None:
        if not self._client.server_is_ready():
            self.get_logger().warning('Button pressed, but no VisitorDetected server is running')
            return
        self.get_logger().info(
            f'Button pressed: visitor at {self._goal.distance:.2f} m, '
            f'{round(math.degrees(self._goal.angle))} deg')
        self._client.send_goal_async(self._goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Receptionist rejected the visitor')
            return
        self.get_logger().info('Visit started')
        goal_handle.get_result_async().add_done_callback(self._on_visit_over)

    def _on_visit_over(self, future) -> None:
        response = future.result()
        self.get_logger().info(
            f'Visit over (goal status {response.status}): {response.result.message!r}')


def main(args=None):
    """Run the node."""
    return run(ButtonVisitorTriggerNode, args)


if __name__ == '__main__':
    sys.exit(main())
