"""
Visitor detector node: starts a visit when a visitor stands in front of the robot.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

from dataclasses import dataclass
import math
import sys

from driving_interfaces.action import VisitorDetected
from driving_service.action_peer import cancel_if_accepted, server_gone, server_identity
from driving_service.params import check, declare_read_only, value
from driving_service.run_node import run
from driving_service.visitor_detector import DetectionParams, VisitorCandidate, VisitorDetector
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# seconds between checks of the receptionist: answer overdue, still the server of the visit
PEER_CHECK_PERIOD = 0.5


@dataclass
class PendingRequest:
    """A goal request the receptionist hasn't answered yet."""

    future: object  # future of send_goal_async
    deadline: float  # when we stop waiting for the answer


class VisitorDetectorNode(Node):
    """Clusters /scan and sends a VisitorDetected goal per visit."""

    def __init__(self, **kwargs):
        super().__init__('VisitorDetector', **kwargs)
        declare_read_only(self, [
            ('action_name', 'visitorDetected',
             'VisitorDetected action server, served by the receptionist main loop'),
            ('scan_topic', '/scan', 'LaserScan topic of the LiDAR'),
            ('receptionist_response_timeout', 5.0,
             'Seconds to wait for the receptionist to accept or reject a visitor'),
            ('detection_distance_min', 0.8,
             'Closest mean distance of a visitor cluster in meters'),
            ('detection_distance_max', 2.5,
             'Farthest mean distance of a visitor cluster in meters'),
            ('detection_angle_range', 60.0,
             'Width of the front window in degrees, centered on lidar_front_angle'),
            ('lidar_front_angle', -math.pi / 2,
             'Scan angle in radians that points straight ahead of the robot'),
            ('person_width_min', 0.35, 'Narrowest cluster that counts as a person in meters'),
            ('person_width_max', 0.7, 'Widest cluster that counts as a person in meters'),
            ('min_confidence', 0.75, 'Lowest confidence (0..1) of a visitor candidate'),
            ('stability_threshold', 8,
             'Processed scans in the stability window, all but 2 need a candidate'),
            ('lidar_frame_skip_rate', 3, 'Process only every n-th scan'),
            ('visitor_timeout', 10.0,
             'Seconds without a candidate after a visit before the cooldown starts'),
            ('cooldown_period', 5.0,
             'Seconds after the scene is clear before the next visitor is detected'),
        ])
        self._detector = VisitorDetector(DetectionParams(
            min_distance=value(self, 'detection_distance_min'),
            max_distance=value(self, 'detection_distance_max'),
            angle_range=value(self, 'detection_angle_range'),
            person_width_min=value(self, 'person_width_min'),
            person_width_max=value(self, 'person_width_max'),
            stability_threshold=value(self, 'stability_threshold'),
            visitor_timeout=value(self, 'visitor_timeout'),
            cooldown_period=value(self, 'cooldown_period'),
            min_confidence=value(self, 'min_confidence'),
            frame_skip_rate=value(self, 'lidar_frame_skip_rate'),
            lidar_front_angle=value(self, 'lidar_front_angle'),
        ))
        self._action_name = value(self, 'action_name')
        self._client = ActionClient(self, VisitorDetected, self._action_name)
        self._response_timeout = value(self, 'receptionist_response_timeout')
        check([(self._response_timeout > 0.0, 'receptionist_response_timeout must be > 0')])
        # PendingRequest for the visitor the receptionist hasn't answered yet (ROS time)
        self._pending = None
        # (goal handle, server identity) of the visit the receptionist runs right now
        self._open_visit = None
        self._peer_timer = self.create_timer(PEER_CHECK_PERIOD, self._check_receptionist)
        self._scan_sub = self.create_subscription(
            LaserScan, value(self, 'scan_topic'), self._on_scan, qos_profile_sensor_data)
        self.get_logger().info('Visitor detector started')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_scan(self, msg: LaserScan) -> None:
        visitor = self._detector.process_scan(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            self._now())
        if visitor is not None:
            self._start_visit(visitor)

    def _start_visit(self, visitor: VisitorCandidate) -> None:
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
        self._pending = PendingRequest(future, self._now() + self._response_timeout)
        future.add_done_callback(lambda done: self._on_goal_response(done, identity))

    def _on_goal_response(self, future, identity) -> None:
        goal_handle = future.result()
        if self._pending is None or self._pending.future is not future:
            # answered after we gave up; never leave that visit running
            cancel_if_accepted(goal_handle)
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
        if self._pending is not None and self._now() >= self._pending.deadline:
            self.get_logger().warning('Receptionist did not answer the visitor, trying again')
            self._pending = None
            self._detector.visit_not_started()
        if self._open_visit is None:
            return
        _goal_handle, identity = self._open_visit
        if server_gone(self, self._action_name, identity):
            self.get_logger().warning('Receptionist stopped or restarted, its visit is lost')
            self._open_visit = None
            self._detector.visit_finished(self._now())


def main(args=None):
    """Run the node."""
    return run(VisitorDetectorNode, args)


if __name__ == '__main__':
    sys.exit(main())
