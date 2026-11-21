import sys
from pathlib import Path
import os
from threading import Lock
import numpy as np

import rclpy
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.action.server import ActionServer, ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosidl_runtime_py import message_to_yaml
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from action_msgs.msg import GoalStatus
from speaking_interfaces.action import AudioCreation, AudioListener, AudioPlayer, AudioTranscriber, AudioOrchestrator
from speaking_interfaces.msg import ListenerParams, TranscriberParams, UploadParams

