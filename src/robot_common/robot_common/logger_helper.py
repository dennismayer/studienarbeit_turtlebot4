from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rosidl_runtime_py import message_to_yaml


def log_goal(node: Node, goal_handle: ServerGoalHandle):
    """Log the received action goal.

    Args:
        node: ROS node used to write the goal logs.
        goal_handle: Server goal handle containing the goal request.

    Returns:
        The goal request.
    """
    
    node.get_logger().debug(f"Received goal:\n{message_to_yaml(goal_handle.request)}")
    return goal_handle.request