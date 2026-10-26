"""Live view for the real-LiDAR test: once a second, the best person-like candidate in front."""

import math
import time

from driving_service.visitor_detector import DetectionParams, VisitorDetector
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# the defaults of config/ros_params.yaml
PARAMS = DetectionParams(
    min_distance=0.8, max_distance=2.0, angle_range=60.0,
    person_width_min=0.15, person_width_max=0.55, stability_threshold=8,
    visitor_timeout=10.0, cooldown_period=5.0, min_confidence=0.0,  # 0: show weak ones too
    frame_skip_rate=1, lidar_front_angle=-math.pi / 2)


class LiveView(Node):

    def __init__(self):
        super().__init__('realtest_live_view')
        self._detector = VisitorDetector(PARAMS)
        self._last = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        now = time.monotonic()
        if now - self._last < 1.0:
            return
        self._last = now
        best = self._detector._best_candidate(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        if best is None:
            seen = 'no person-like candidate in the front window'
        else:
            flag = 'counts' if best.confidence >= 0.75 else 'too weak'
            seen = (f'candidate {best.distance:.2f} m {math.degrees(best.angle):+.1f} deg '
                    f'confidence {best.confidence:.2f} ({flag})')
        print(f'{time.strftime("%H:%M:%S")} {seen}', flush=True)


rclpy.init()
rclpy.spin(LiveView())
