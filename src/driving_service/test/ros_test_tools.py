"""Helpers shared by the tests that start ROS."""

import threading
import time

from rclpy.executors import MultiThreadedExecutor

# A domain of its own and no discovery server, so no test can ever reach the
# robot's real nodes: the robot's Fast DDS discovery server (ROS_DISCOVERY_SERVER) connects
# participants across domain IDs, so the domain ID alone does not isolate.
TEST_DOMAIN_ID = 87
TIMEOUT = 10.0


def wait_for(future, timeout=TIMEOUT):
    """Return the result of a future, failing the test when it takes too long."""
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, 'timed out'
        time.sleep(0.01)
    return future.result()


def wait_until(condition, timeout=TIMEOUT):
    """Wait for a condition, return whether it came true (late messages get a moment)."""
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.01)
    return condition()


class SpinsAlone:
    """
    Fake action server node that runs in an executor of its own.

    The fake sets self._server (its ActionServer), self._executor = None and
    self.stopped (a threading.Event). A fake that is never spun answers
    nothing, like a crashed process that is still in the graph.
    """

    def spin_in_background(self):
        """Spin in an executor of its own, so stop() can end it like a process."""
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        return self

    def stop(self):
        """Vanish like a crashed process: running goals never get a result."""
        if self.stopped.is_set():
            return
        # destroying the action server inside a spinning executor can kill its spin
        # thread, so end the executor first, then the node, like a dying process
        self.stopped.set()
        if self._executor is not None:
            self._executor.shutdown()
        self._server.destroy()  # destroy_node() leaves action servers alive
        self.destroy_node()
