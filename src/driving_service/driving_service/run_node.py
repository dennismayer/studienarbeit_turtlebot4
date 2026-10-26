"""
Shared entry point of the driving_service nodes.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

import signal
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import try_shutdown


# seconds per spin step, i.e. the longest Ctrl-C waits before it is noticed
SPIN_STEP = 0.1

_stop_requested = threading.Event()


def run(node_class, args=None) -> int:
    """
    Spin one node in a multi-threaded executor until Ctrl-C or shutdown.

    rclpy's own SIGINT handler would shut ROS down before we see Ctrl-C, so it
    is off: Ctrl-C only asks the spin loop to stop while ROS still works, and a
    node with a stop(executor) method can wind down first, e.g. cancel goals.
    The handler doesn't raise: an exception thrown into spin_once can leave the
    executor marked as spinning, and stop() could not spin it any more.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # SIGTERM (ros2 launch escalates to it, systemd stops with it) winds down like Ctrl-C;
    # SIGINT explicitly too, it may have been ignored by whoever started us (e.g. `&`)
    signal.signal(signal.SIGTERM, _request_stop)
    signal.signal(signal.SIGINT, _request_stop)
    node = None
    executor = None
    exit_code = 0
    try:
        node = node_class()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        # short steps: signal handlers only run between them, not inside rcl_wait
        while rclpy.ok() and not _stop_requested.is_set():
            executor.spin_once(timeout_sec=SPIN_STEP)
        if _stop_requested.is_set():
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


def _request_stop(_signum, _frame):
    _stop_requested.set()
