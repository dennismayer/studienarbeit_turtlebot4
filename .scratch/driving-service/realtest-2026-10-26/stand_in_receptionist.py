"""Stand-in receptionist for the real-LiDAR test: prints each Visitor, ends the Visit after 3 s."""

import math
import sys
import time

from driving_interfaces.action import VisitorDetected
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class StandIn(Node):

    def __init__(self, action_name):
        super().__init__('realtest_stand_in_receptionist')
        self._server = ActionServer(self, VisitorDetected, action_name, self._execute)
        self.get_logger().info(f'serving {action_name}')

    def _execute(self, goal_handle):
        g = goal_handle.request
        print(f'{time.strftime("%H:%M:%S")} VISITOR distance={g.distance:.2f} m '
              f'angle={math.degrees(g.angle):+.1f} deg (left +) confidence={g.confidence:.2f}',
              flush=True)
        time.sleep(3.0)
        goal_handle.succeed()
        print(f'{time.strftime("%H:%M:%S")} visit over', flush=True)
        return VisitorDetected.Result(success=True, message='stand-in')


rclpy.init()
rclpy.spin(StandIn(sys.argv[1]))
