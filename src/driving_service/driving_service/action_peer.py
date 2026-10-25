"""
Tell whether the server behind an action client is still the same one.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

from rclpy.node import Node


def server_identity(node: Node, action_name: str) -> frozenset:
    """
    Identify the action server by the endpoint GIDs of its status publisher.

    Empty when no server is running. A restarted server has new GIDs, so a
    change means every goal the old server had is lost.
    """
    topic = node.resolve_topic_name(action_name) + '/_action/status'
    return frozenset(
        bytes(info.endpoint_gid) for info in node.get_publishers_info_by_topic(topic))
