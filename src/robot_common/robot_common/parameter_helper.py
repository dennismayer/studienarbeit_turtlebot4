from rclpy.node import Node

from typing import TypeVar

T = TypeVar("T")

def get_param(node: Node, name: str, expected: type[T]) -> T:
    value = node.get_parameter(name).value
    if not isinstance(value, expected):
        raise TypeError(f"Parameter '{name}' is {type(value).__name__}, expected {expected.__name__}")
    return value