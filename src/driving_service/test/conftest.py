"""Fixtures for the tests that start ROS."""

import pytest
import rclpy
from ros_test_tools import TEST_DOMAIN_ID


@pytest.fixture
def ros(monkeypatch):
    """ROS in a domain of its own, without the robot's discovery server."""
    monkeypatch.delenv('ROS_DISCOVERY_SERVER', raising=False)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init(domain_id=TEST_DOMAIN_ID)
    yield
    rclpy.try_shutdown()
