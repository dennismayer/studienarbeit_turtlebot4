"""
Visitor detector node: starts a visit when a visitor stands in front of the robot.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

import math
import sys

from driving_interfaces.action import VisitorDetected
from driving_service.action_peer import server_identity
from driving_service.run_node import run
from driving_service.visitor_detector import DetectionParams, Visitor, VisitorDetector
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _descriptor(description, parameter_type):
    return ParameterDescriptor(description=description, read_only=True, type=parameter_type)


# seconds between checks whether the receptionist still has the open visit
PEER_CHECK_PERIOD = 0.5

_DOUBLE = ParameterType.PARAMETER_DOUBLE
_INTEGER = ParameterType.PARAMETER_INTEGER
_STRING = ParameterType.PARAMETER_STRING


class VisitorDetectorNode(Node):
    """Clusters /scan and sends a VisitorDetected goal per visit."""

    def __init__(self, **kwargs):
        super().__init__('VisitorDetector', **kwargs)
        self.declare_parameters('', [
            ('action_name', 'visitorDetected', _descriptor(
                'VisitorDetected action server, served by the receptionist main loop', _STRING)),
            ('scan_topic', '/scan', _descriptor('LaserScan topic of the LiDAR', _STRING)),
            ('receptionist_response_timeout', 5.0, _descriptor(
                'Seconds to wait for the receptionist to accept or reject a visitor', _DOUBLE)),
            ('detection_distance_min', 0.8, _descriptor(
                'Closest mean distance of a visitor cluster in meters', _DOUBLE)),
            ('detection_distance_max', 2.5, _descriptor(
                'Farthest mean distance of a visitor cluster in meters', _DOUBLE)),
            ('detection_angle_range', 60.0, _descriptor(
                'Width of the front window in degrees, centered on lidar_front_angle', _DOUBLE)),
            ('lidar_front_angle', -math.pi / 2, _descriptor(
                'Scan angle in radians that points straight ahead of the robot', _DOUBLE)),
            ('person_width_min', 0.35, _descriptor(
                'Narrowest cluster that counts as a person in meters', _DOUBLE)),
            ('person_width_max', 0.7, _descriptor(
                'Widest cluster that counts as a person in meters', _DOUBLE)),
            ('min_confidence', 0.75, _descriptor(
                'Lowest confidence (0..1) of a visitor candidate', _DOUBLE)),
            ('stability_threshold', 8, _descriptor(
                'Processed scans in the stability window, all but 2 need a candidate',
                _INTEGER)),
            ('lidar_frame_skip_rate', 3, _descriptor(
                'Process only every n-th scan', _INTEGER)),
            ('visitor_timeout', 10.0, _descriptor(
                'Seconds without a candidate after a visit before the cooldown starts', _DOUBLE)),
            ('cooldown_period', 5.0, _descriptor(
                'Seconds after the scene is clear before the next visitor is detected', _DOUBLE)),
        ])
        param = self.get_parameter
        self._detector = VisitorDetector(DetectionParams(
            min_distance=param('detection_distance_min').value,
            max_distance=param('detection_distance_max').value,
            angle_range=param('detection_angle_range').value,
            person_width_min=param('person_width_min').value,
            person_width_max=param('person_width_max').value,
            stability_threshold=param('stability_threshold').value,
            visitor_timeout=param('visitor_timeout').value,
            cooldown_period=param('cooldown_period').value,
            min_confidence=param('min_confidence').value,
            frame_skip_rate=param('lidar_frame_skip_rate').value,
            lidar_front_angle=param('lidar_front_angle').value,
        ))
        self._action_name = param('action_name').value
        self._client = ActionClient(self, VisitorDetected, self._action_name)
        self._response_timeout = param('receptionist_response_timeout').value
        if self._response_timeout <= 0.0:
            raise ValueError('receptionist_response_timeout must be > 0')
        # (send future, deadline) of the visitor the receptionist hasn't answered yet
        self._pending = None
        # (goal handle, server identity) of the visit the receptionist runs right now
        self._open_visit = None
        self._peer_timer = self.create_timer(PEER_CHECK_PERIOD, self._check_receptionist)
        self._scan_sub = self.create_subscription(
            LaserScan, param('scan_topic').value, self._on_scan, qos_profile_sensor_data)
        self.get_logger().info('Visitor detector started')

    def receptionist_ready(self) -> bool:
        """Whether the VisitorDetected server is up."""
        return self._client.server_is_ready()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_scan(self, msg: LaserScan) -> None:
        visitor = self._detector.process_scan(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, self._now())
        if visitor is not None:
            self._start_visit(visitor)

    def _start_visit(self, visitor: Visitor) -> None:
        if not self._client.server_is_ready():
            self.get_logger().warning(
                'Visitor detected, but no VisitorDetected server is running',
                throttle_duration_sec=10.0)
            self._detector.visit_not_started()
            return
        self.get_logger().info(
            f'Visitor at {visitor.distance:.2f} m, {round(math.degrees(visitor.angle))} deg '
            f'(confidence {visitor.confidence:.2f})')
        goal = VisitorDetected.Goal(
            distance=visitor.distance, angle=visitor.angle, confidence=visitor.confidence)
        identity = server_identity(self, self._action_name)
        future = self._client.send_goal_async(goal)
        self._pending = (future, self._now() + self._response_timeout)
        future.add_done_callback(lambda done: self._on_goal_response(done, identity))

    def _on_goal_response(self, future, identity) -> None:
        goal_handle = future.result()
        if self._pending is None or self._pending[0] is not future:
            # answered after we gave up; never leave that visit running
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self._pending = None
        if not goal_handle.accepted:
            self.get_logger().info('Receptionist rejected the visitor')
            self._detector.visit_finished(self._now())
            return
        self._open_visit = (goal_handle, identity)
        goal_handle.get_result_async().add_done_callback(
            lambda result_future: self._on_visit_over(result_future, goal_handle))

    def _on_visit_over(self, future, goal_handle) -> None:
        if self._open_visit is None or self._open_visit[0] is not goal_handle:
            return  # already given up on this visit
        self._open_visit = None
        response = future.result()
        self.get_logger().info(
            f'Visit over (goal status {response.status}): {response.result.message!r}')
        self._detector.visit_finished(self._now())

    def _check_receptionist(self) -> None:
        if self._pending is not None and self._now() >= self._pending[1]:
            self.get_logger().warning('Receptionist did not answer the visitor, trying again')
            self._pending = None
            self._detector.visit_not_started()
        if self._open_visit is None:
            return
        _goal_handle, identity = self._open_visit
        if server_identity(self, self._action_name) != identity:
            self.get_logger().warning('Receptionist stopped or restarted, its visit is lost')
            self._open_visit = None
            self._detector.visit_finished(self._now())


def main(args=None):
    """Run the node."""
    return run(VisitorDetectorNode, args)


if __name__ == '__main__':
    sys.exit(main())
