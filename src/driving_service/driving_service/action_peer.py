"""
Dealing with the action server on the other side: is it still there, late answers.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

from rclpy.node import Node


def cancel_if_accepted(goal_handle) -> None:
    """Cancel a goal that was accepted after we stopped waiting for it."""
    if goal_handle is not None and goal_handle.accepted:
        goal_handle.cancel_goal_async()


def server_identity(node: Node, action_name: str) -> frozenset:
    """
    Identify the action server by the endpoint GIDs of its status publisher.

    Empty when no server is running. A restarted server has new GIDs. A crashed
    server can stay in the graph for a while (about 20 s after kill -9), so the
    identity may hold dead copies next to the live server.
    """
    topic = node.resolve_topic_name(action_name) + '/_action/status'
    return frozenset(
        bytes(info.endpoint_gid) for info in node.get_publishers_info_by_topic(topic))


def server_gone(node: Node, action_name: str, identity: frozenset) -> bool:
    """
    Whether every server endpoint of identity has left the graph.

    Only then is a goal sent at that time surely lost: a dead copy dropping out
    while the server that runs the goal stays doesn't count.
    """
    return not (server_identity(node, action_name) & identity)
