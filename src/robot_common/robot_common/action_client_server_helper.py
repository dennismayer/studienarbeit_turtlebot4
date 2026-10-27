from rclpy.node import Node
from threading import Lock
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.action.server import ActionServer, ServerGoalHandle, GoalResponse, CancelResponse

class GoalCanceled(Exception):
    pass

class GoalAbort(Exception):
    pass

class GoalRejected(Exception):
    pass

def check_if_client_started(client: ActionClient, timeout: float = 45.0):
    """Wait for the action server to become available or raise a RuntimeError.

    Args:
        client: The action client to check.
        timeout: Maximum time to wait for the action server, in seconds.
    """
    
    if client.wait_for_server(timeout_sec=timeout):
        return 
    raise RuntimeError(f"Action not reachable: {client._action_name}")

async def send_universal_goal(node: Node, client: ActionClient, goal, action_timeout: float, feedback_callback=None) -> ClientGoalHandle:
    if not client.server_is_ready():
        raise GoalAbort(f"Worker {client._action_name} is not available")

    # goal acceptance should be quick, so the short action_timeout fits here
    node.get_logger().debug(f"Sending goal to {client._action_name}")
    send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
    worker_handle: ClientGoalHandle | None = await await_with_timeout(node, send_future, action_timeout)
    if send_future.cancelled():
        raise GoalAbort(f"Worker {client._action_name} did not respond to the goal within {action_timeout:.1f} s")
    if worker_handle is None or not worker_handle.accepted:
        raise GoalRejected(f"Worker {client._action_name} rejected the goal (busy?)")

    return worker_handle

async def wait_for_universal_result(node: Node, worker_handle: ClientGoalHandle, timeout_sec: float):
    action_name = worker_handle._action_client._action_name

    result_future = worker_handle.get_result_async()
    response = await await_with_timeout(node, result_future, timeout_sec)
    if result_future.cancelled():
        # not awaited: a dead worker would never answer the cancel request either
        worker_handle.cancel_goal_async()
        raise GoalAbort(f"Worker {action_name} did not finish within {timeout_sec:.1f} s, goal canceled")

    if response is None:
        raise GoalAbort(f"Worker {action_name} returned no result")

    return response

async def await_with_timeout(node: Node, future, timeout_sec: float):
    def on_timeout():
        timer.cancel()
        future.cancel()          # wakes up the await below with None

    timer = node.create_timer(timeout_sec, on_timeout)
    try:
        return await future
    finally:
        timer.cancel()
        node.destroy_timer(timer)
