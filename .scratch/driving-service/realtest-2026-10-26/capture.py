"""Record raw /scan messages for N seconds into a .npz for offline tuning."""

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

out, seconds = sys.argv[1], float(sys.argv[2])
rclpy.init()
node = Node('realtest_capture')
scans, meta = [], {}


def on_scan(msg):
    scans.append(np.array(msg.ranges, dtype=float))
    meta.update(angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                range_min=msg.range_min, range_max=msg.range_max)


node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
end = time.monotonic() + seconds
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
np.savez_compressed(out, ranges=np.array(scans, dtype=np.float32), **meta)
print(f'{len(scans)} scans -> {out}')
