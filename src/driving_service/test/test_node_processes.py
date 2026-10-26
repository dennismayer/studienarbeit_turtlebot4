"""The node processes, started the way ros2 run starts them."""

import subprocess
import sys

import pytest
from ros_test_tools import isolated_env


@pytest.mark.parametrize('module, bad_param, message', [
    ('driving_service.NavigatorNode', 'nav_retry_attempts:=0',
     'nav_retry_attempts must be >= 1'),
    ('driving_service.VisitorDetectorNode', 'stability_threshold:=1',
     'stability_threshold must be >= 3'),
])
def test_node_with_an_invalid_parameter_exits_with_an_error(module, bad_param, message):
    process = subprocess.run(
        [sys.executable, '-m', module, '--ros-args', '-p', bad_param],
        env=isolated_env(), capture_output=True, text=True, timeout=30)

    assert process.returncode != 0
    assert message in process.stderr
