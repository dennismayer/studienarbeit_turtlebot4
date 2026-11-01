"""Fixtures for the tests that start ROS."""

import os

import pytest
import rclpy
from ros_test_tools import isolated_env, TEST_DOMAIN_ID


def pytest_collection_modifyitems(items):
    """
    Run the linters before every other test.

    flake8 forks worker processes; forking once ROS tests have left DDS threads
    behind can deadlock the workers and hang the whole run.
    """
    items.sort(key=lambda item: item.get_closest_marker('linter') is None)


@pytest.fixture
def ros(monkeypatch):
    """ROS in a domain of its own, without the robot's discovery server."""
    env = isolated_env()
    for name in set(os.environ) - set(env):
        monkeypatch.delenv(name)
    for name, value in env.items():
        monkeypatch.setenv(name, value)
    rclpy.init(domain_id=TEST_DOMAIN_ID)
    yield
    rclpy.try_shutdown()
