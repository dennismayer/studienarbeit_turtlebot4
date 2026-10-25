"""
Shared entry point of the driving_service nodes.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.utilities import try_shutdown


def run(node_class, args=None) -> int:
    """Spin one node in a multi-threaded executor until Ctrl-C or shutdown."""
    rclpy.init(args=args)
    node = None
    executor = None
    exit_code = 0
    try:
        node = node_class()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print('\nShutting down...', file=sys.stderr)
    except Exception as e:
        print(f'Exception occurred in main thread: {e}', file=sys.stderr)
        exit_code = -1
    finally:
        # shut ROS down before the executor, so callbacks waiting on ROS give up
        try_shutdown()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
    return exit_code
