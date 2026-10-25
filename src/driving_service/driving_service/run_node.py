"""
Shared entry point of the driving_service nodes.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import try_shutdown


# seconds per spin step, i.e. the longest Ctrl-C waits before it is noticed
SPIN_STEP = 0.1


def run(node_class, args=None) -> int:
    """
    Spin one node in a multi-threaded executor until Ctrl-C or shutdown.

    rclpy's own SIGINT handler would shut ROS down before we see Ctrl-C, so it
    is off: Ctrl-C arrives as KeyboardInterrupt while ROS still works, and a
    node with a stop(executor) method can wind down first, e.g. cancel goals.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    exit_code = 0
    try:
        node = node_class()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        # short steps: Python only raises KeyboardInterrupt between them, not inside rcl_wait
        while rclpy.ok():
            executor.spin_once(timeout_sec=SPIN_STEP)
    except KeyboardInterrupt:
        print('\nShutting down...', file=sys.stderr)
        stop = getattr(node, 'stop', None)
        if stop is not None:
            stop(executor)
    except ExternalShutdownException:
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
