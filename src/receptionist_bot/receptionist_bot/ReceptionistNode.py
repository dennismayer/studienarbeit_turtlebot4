import sys
from pathlib import Path
import os
from threading import Lock
import numpy as np
from typing import TYPE_CHECKING, Mapping, Any, Sequence
from dotenv import load_dotenv
import yaml

import boto3
from botocore.config import Config


import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.action.server import ActionServer, ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rosidl_runtime_py import message_to_yaml
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


from action_msgs.msg import GoalStatus
from speaking_interfaces.action import AudioOrchestrator
from speaking_interfaces.msg import ListenerParams, TranscriberParams, UploadParams
from driving_interfaces.action import MoveToLocation, VisitorDetected

from robot_common.action_client_server_helper import check_if_client_started, send_universal_goal, wait_for_universal_result, GoalAbort, GoalCanceled, GoalRejected

if TYPE_CHECKING:
    from mypy_boto3_bedrock_runtime.client import BedrockRuntimeClient
    from mypy_boto3_bedrock_runtime.type_defs import (
        ConverseResponseTypeDef,
        MessageUnionTypeDef,
        ToolTypeDef,
    )

DEFAULT_NODE_NAME = 'Receptionist'
PACKAGE_NAME = 'receptionist_bot'

# parameters for config yaml
AUDIO_ORCHESTRATOR_ACTION = 'speechRequest'
NAVIGATOR_ACTION = 'moveToLocation'
VISITOR_ACTION = 'visitorDetected'

TIMEOUT_START_CLIENT = 30.0
ENV_FILE_PATH = "/home/ubuntu/turtlebot4_ws/.env"
DIALOG_SCRIPT_NAME = "short_test_script.yaml"
ACTION_TIMEOUT = 5.0
ORCHESTRATOR_TIMEOUT = 120.0


class ReceptionistNode(Node):
    def __init__(self):
        super().__init__(DEFAULT_NODE_NAME)
        
        # Startup procedure
        self._declare_node_parameters()
        self._declare_node_variables()
        self._start_action_clients()
        self._start_action_server()
    
    def _declare_node_parameters(self):
        pass
    
    def _declare_node_variables(self):
        if not load_dotenv(ENV_FILE_PATH):
            raise FileExistsError(f"No .env file at: {ENV_FILE_PATH}")
        
        script_path = Path(get_package_share_directory(PACKAGE_NAME)) / PACKAGE_NAME / "dialog_scripts" / DIALOG_SCRIPT_NAME
        if not os.path.isfile(script_path):
            raise FileExistsError(f"No dialog file at: {script_path}")
        
        with script_path.open("r", encoding="utf-8") as file:
            self._dialog_script = yaml.safe_load(file)
        self._check_script_file()
        
        self._bedrock_client: "BedrockRuntimeClient" = boto3.client(
            "bedrock-runtime",
            region_name="eu-central-1",
            config=Config(
                connect_timeout=5,
                read_timeout=15,                 # a short dialog reply takes ~1 s; don't let the robot hang for an hour
                retries={"max_attempts": 2},
            )
        )
        
        self._action_timeout = ACTION_TIMEOUT
        self._orchestrator_timeout = ORCHESTRATOR_TIMEOUT
        self._active_worker_handle = None
        self._lock_worker_handle = Lock()
        self._lock_callback = Lock()
    
    def _start_action_clients(self) -> None:
        self.get_logger().info("Starting Action Clients ...")
        
        self.__audio_orchestrator_client = ActionClient(
            self,
            AudioOrchestrator,
            AUDIO_ORCHESTRATOR_ACTION
        )
        
        self.__driving_navigator_client = ActionClient(
            self, 
            MoveToLocation,
            NAVIGATOR_ACTION
        )
        
        check_if_client_started(self.__audio_orchestrator_client, TIMEOUT_START_CLIENT)
        check_if_client_started(self.__driving_navigator_client, TIMEOUT_START_CLIENT)
        self.get_logger().info("All worker action servers reachable")
    
    def _start_action_server(self) -> None:
        action_name = VISITOR_ACTION
        self.__action_server = ActionServer(
            node=self,
            action_type=VisitorDetected,
            action_name=action_name,
            execute_callback=self._execute_callback,
            cancel_callback=self._cancel_callback,
            goal_callback=self._goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_logger().info(f"Action server '{action_name}' ready, waiting for speech requests")
    
    async def _execute_callback(self, goal_handle: ServerGoalHandle) -> VisitorDetected.Result:
        # define local variables
        try:
            retry = False
            available_messages = self._dialog_script["messages"]
            next_message = available_messages[self._dialog_script["start"]]
            memory = {}
            
            while next_message is not None:   
                # todo check for cancel 
                # todo possibility for moving tool
                # todo possibility for getting all professors
                # todo counter for retries
                if "end" in next_message and next_message["end"]:
                    goal = AudioOrchestrator.Goal()
                    goal.question_or_sentence = next_message["say"].format(**memory)
                    goal.response_handling = AudioOrchestrator.Goal.NO_RESPONSE
                    
                    await self._run_worker(self.__audio_orchestrator_client, goal, timeout_sec=self._orchestrator_timeout)
                    
                    next_message = None
                    
                    continue
                
                if retry:
                    message_str = next_message["retry_say"]
                else:
                    message_str = next_message["say"]
                
                retry = False
                    
                goal = AudioOrchestrator.Goal()
                goal.question_or_sentence = message_str.format(**memory)
                goal.response_handling = AudioOrchestrator.Goal.TRANSCRIBE_RESPONSE
                
                orchestrator_result = await self._run_worker(self.__audio_orchestrator_client, goal, timeout_sec=self._orchestrator_timeout)
                response_text = orchestrator_result.transcribed_text
                
                message = {
                    "role": "user",
                    "content": [{"text": response_text}]
                }
                
                system_prompt = self._dialog_script["base_prompt"] + "\n" + next_message["prompt"]
                
                extract_list, transition_list, tool = self._define_Tool(next_message)
                
                response = self._call_llm_api(system_prompt, [message], tool)
                
                if "message" not in response["output"]:
                    raise ValueError("No message field in response")
                
                # the structured answer arrives as an already parsed dict in the toolUse block, not as text
                tool_use = next((block["toolUse"] for block in response["output"]["message"]["content"] if "toolUse" in block), None)
                if response["stopReason"] == "tool_use" and tool_use is not None:
                    reply = tool_use["input"]
                else:
                    raise ValueError(f"No valid response: {response["output"]["message"]}")
                
                self.get_logger().info(f"Reply from LLM: {reply}")
                
                if reply["choice"] not in transition_list:
                    raise ValueError(f"{reply["choice"]} not a valid answer option ({transition_list})")
                
                if reply["choice"] == "unclear":
                    next_message = available_messages[next_message["fallback"]]
                    retry = True
                    continue
                
                next_message_name = next((block["goto"] for block in next_message["transitions"] if block["if"] == reply["choice"]), None)
                next_message = available_messages[next_message_name] if next_message_name is not None else None
                
                for extraction in extract_list:
                    memory[extraction] = reply[extraction]
                    self.get_logger().info(f"Saved memory: {extraction}: {reply[extraction]}")
            
            goal_handle.succeed()
            return self._assemble_result(True, "Finished")
        
        except GoalCanceled:
            return self._handle_cancel_callback(goal_handle)
        
        except GoalAbort as e:
            return self._handle_error_callback(goal_handle, f"{e}")
        
        except GoalRejected as e:
            return self._handle_error_callback(goal_handle, f"{e}")

        except Exception as e:
            return self._handle_error_callback(goal_handle, f"An Unknown Error occurred: {e}")
        finally:
            with self._lock_worker_handle:
                if self._active_worker_handle is not None:
                    self._active_worker_handle.cancel_goal_async()
                    self._active_worker_handle = None
    
    def _goal_callback(self, goal_request):
        if not self._lock_callback.acquire(blocking=False):
            self.get_logger().warning("Rejected speech request: another request is still running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def _check_script_file(self):
        pass
    
    def _define_Tool(self, message: Mapping[str, Any]):
        input_properties = {}
        required = []
        extract_ids = []
        
        if "extract" in message:
            extract = message["extract"]
            for entry in extract:
                input_properties[entry] = {"type": "string", "description": extract[entry]}
                required.append(entry)
                extract_ids.append(entry)
        
        transitions_list = []
        description_list = ["Gewählte Option:"]
        if "transitions" in message:
            transitions = message["transitions"]
            for transition in transitions:
                transitions_list.append(transition["if"])
                description_list.append(f"{transition["if"]}: {transition["description"]}")
            
            transitions_list.append("unclear")
            description_list.append("unclear: Antwort passt zu keiner Option oder ist unverständlich")    
            input_properties["choice"] = {
                "type": "string",
                "enum": transitions_list,
                "description": "\n- ".join(description_list)
            }
            required.append("choice")
        
        
        return (extract_ids, transitions_list,
            {
            "toolSpec": {
                "name": "forced_reply",
                "description": "Gib die Antwort des Empfangsroboters.",
                "inputSchema": {
                    "json": {
                        "type": "object",
                        "properties": input_properties,
                        "required": required,
                    }
                },
            }
        })
    
    def _call_llm_api(self, system_prompt: str, messages: Sequence, reply_tool):
        return self._bedrock_client.converse(
            modelId="eu.amazon.nova-2-lite-v1:0",
            system=[
                {
                    "text": system_prompt
                }
            ],
            messages=messages,
            inferenceConfig={"maxTokens": 300},
            toolConfig={
                "tools": [reply_tool],
                "toolChoice": {"tool": {"name": reply_tool["toolSpec"]["name"]}},   # force the structured answer
            },
        )
    
    async def _send_goal(self, client: ActionClient, goal, feedback_callback=None) -> ClientGoalHandle:
        worker_handle = await send_universal_goal(self, client, goal, self._action_timeout, feedback_callback=feedback_callback)

        with self._lock_worker_handle:
            self._active_worker_handle = worker_handle
        return worker_handle
    
    async def _wait_for_result(self, worker_handle: ClientGoalHandle, timeout_sec: float):
        try:
            response = await wait_for_universal_result(self, worker_handle, timeout_sec)
        finally:
            with self._lock_worker_handle:
                self._active_worker_handle = None

        action_name = worker_handle._action_client._action_name
        
        match response.status:
            case GoalStatus.STATUS_SUCCEEDED:
                return response.result
            case GoalStatus.STATUS_CANCELED:
                self.get_logger().debug(f"Worker {action_name} confirmed the cancel")
                raise GoalCanceled()
            case GoalStatus.STATUS_ABORTED:
                raise GoalAbort(f"Worker {action_name} aborted: {response.result.message}")
            case _:
                raise GoalAbort(f"Worker {action_name} finished with unexpected status {response.status}")

    async def _run_worker(self, client: ActionClient, goal, feedback_callback=None, *, timeout_sec: float):
        worker_handle = await self._send_goal(client, goal, feedback_callback)
        return await self._wait_for_result(worker_handle, timeout_sec=timeout_sec)
    
    def _assemble_result(self, success: bool, message: str):
            result = VisitorDetected.Result()
            result.success = success
            result.message = message
            return result
    
    def _cancel_callback(self, cancel_request):
            self.get_logger().info("Cancel requested by client")
            with self._lock_worker_handle:
                if self._active_worker_handle is not None:
                    self._active_worker_handle.cancel_goal_async()
            return CancelResponse.ACCEPT 
        
    def _handle_error_callback(self, goal_handle, error_msg):
        self.get_logger().error(f"Request aborted: {error_msg}")
        goal_handle.abort()
        return self._assemble_result(False, error_msg)
    
    def _handle_cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Request canceled")
        goal_handle.canceled()
        return self._assemble_result(False, "Canceled")
    
    def destroy_node(self):
        self._bedrock_client.close()
    

def main(args=None):
    rclpy.init(args=args)

    receptionist_node = None
    executor = None
    exit_code = 0

    try:
        receptionist_node = ReceptionistNode()
        executor = MultiThreadedExecutor()
        executor.add_node(receptionist_node)

        executor.spin()
    except RuntimeError as e:
        print(e, file=sys.stderr)
        exit_code = -1
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...", file=sys.stderr)
    except Exception as e:
        print(f"Exception occurred in main thread: {e}", file=sys.stderr)
        exit_code = -1
    finally:
        if executor is not None:
            executor.shutdown()
        if receptionist_node is not None:
            receptionist_node.destroy_node()
        try_shutdown()

    return exit_code

if __name__ == "__main__":
    sys.exit(main())