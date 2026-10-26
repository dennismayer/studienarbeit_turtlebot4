"""
Read-only ROS parameters and their checks, shared by the driving_service nodes.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node

_TYPES = {
    str: ParameterType.PARAMETER_STRING,
    int: ParameterType.PARAMETER_INTEGER,
    float: ParameterType.PARAMETER_DOUBLE,
}


def declare_read_only(node: Node, parameters) -> None:
    """Declare (name, default, description) triples; the type follows the default."""
    node.declare_parameters('', [
        (name, default, ParameterDescriptor(
            description=description, read_only=True, type=_TYPES[type(default)]))
        for name, default, description in parameters
    ])


def value(node: Node, name: str):
    """Value of a declared parameter."""
    return node.get_parameter(name).value


def check(checks) -> None:
    """Raise ValueError with the message of the first (ok, message) pair that fails."""
    for ok, message in checks:
        if not ok:
            raise ValueError(message)
