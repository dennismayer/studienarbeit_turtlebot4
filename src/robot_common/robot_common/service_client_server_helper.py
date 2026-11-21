from rclpy.client import Client
from rclpy.node import Node

from robot_common.action_client_server_helper import GoalAbort, await_with_timeout

def check_if_service_client_started(client: Client, timeout: float = 45.0):
    """Wait for the service server to become available or raise a RuntimeError.

    Args:
        client: The service client to check.
        timeout: Maximum time to wait for the service server, in seconds.
    """
    
    if client.wait_for_service(timeout_sec=timeout):
        return 
    raise RuntimeError(f"Service not reachable: {client.srv_name}")

async def run_service_request(node: Node, client: Client, request, timeout_sec=5.0):
    try:
        future = client.call_async(request)
        response = await await_with_timeout(node, future, timeout_sec)
    except Exception as e:
        raise GoalAbort(f"Error in reaching the service {client.srv_name}. Error: {e}")
    
    if response is None:
        raise GoalAbort(f"Didn't get a response from {client.srv_name}")
    
    return response
    