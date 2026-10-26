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
