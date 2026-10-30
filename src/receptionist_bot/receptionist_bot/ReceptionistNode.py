import sys
from enum import Enum
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
MAX_TRY_SPEECH = 3

class RetryError(Exception):
    pass

class MESSAGE_TYPE(Enum):
    STANDARD  = 0
    UPLOAD = 1
    DRIVE = 2
    LOOK_FOR_CONSULTATION_HOUR = 3
    

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
        
        script_path = Path(get_package_share_directory(PACKAGE_NAME)) / "dialog_scripts" / DIALOG_SCRIPT_NAME
        if not os.path.isfile(script_path):
            raise FileExistsError(f"No dialog file at: {script_path}")
        
        with script_path.open("r", encoding="utf-8") as file:
            self._dialog_script = yaml.safe_load(file)
        self._check_script_file()
        
        self._bedrock_client: "BedrockRuntimeClient" = boto3.client(
            'bedrock-runtime',
            region_name="eu-central-1",
            config=Config(
                connect_timeout=5,
                read_timeout=15,                 # a short dialog reply takes ~1 s; don't let the robot hang for an hour
                retries={"max_attempts": 2},
            )
        )
        
        self._action_timeout = ACTION_TIMEOUT
        self._orchestrator_timeout = ORCHESTRATOR_TIMEOUT
        self._active_worker_handle = []
        self._away_from_station = False
        self._lock_worker_handle = Lock()
        self._lock_callback = Lock()
        
        #self.get_logger().info(f"{self._define_Tool(self._dialog_script["messages"]["ask_professor"], ["Dennis Mayer", "Charlotte Braun"])}")
    
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
            self._retry = False
            self._available_messages = self._dialog_script["messages"]
            self._next_message = self._available_messages[self._dialog_script["start"]]
            self._memory = {}
            self._counter_retrys = 0
            
            while self._next_message is not None:   
                # todo check for cancel 
                if "end" in self._next_message and self._next_message["end"]:
                    await self._end_script()
                    continue
                
                match self._determine_message_type():
                    case MESSAGE_TYPE.STANDARD:
                        if await self._react_to_standard_message() == "continue":
                            continue
                    case MESSAGE_TYPE.DRIVE:
                        await self._react_to_drive_message()
                    case MESSAGE_TYPE.UPLOAD:
                        await self._react_to_upload_message()
                    case MESSAGE_TYPE.LOOK_FOR_CONSULTATION_HOUR:
                        pass
            
            goal_handle.succeed()
            return self._assemble_result(True, "Finished")
        
        except GoalCanceled:
            return self._handle_cancel_callback(goal_handle)
        
        except GoalAbort as e:
            return self._handle_error_callback(goal_handle, f"{e}")
        
        except GoalRejected as e:
            return self._handle_error_callback(goal_handle, f"{e}")
        
        except RetryError:
            # say Sorry I can't understand u and will stop this conversation
            return VisitorDetected.Result()

        except Exception as e:
            return self._handle_error_callback(goal_handle, f"An Unknown Error occurred: {e}")
        finally:
            with self._lock_worker_handle:
                if self._active_worker_handle:
                    self.get_logger().debug(f"Canceling {len(self._active_worker_handle)} leftover worker goal(s)")
                for worker_handle in self._active_worker_handle:
                    worker_handle.cancel_goal_async()
                self._active_worker_handle.clear()
            
            if self._away_from_station:
                goal = MoveToLocation.Goal()
                goal.x = 0
                goal.y = 0
                goal.yaw = 0
                
                try:
                    await self._run_worker(self.__driving_navigator_client, goal, timeout_sec=180.0)
                except Exception as e:
                    self.get_logger().error("Counldn't get back to station!")
    
    async def _end_script(self):
        if not self._next_message:
            raise ValueError("next message not defined")
    
        goal = AudioOrchestrator.Goal()
        goal.question_or_sentence = self._next_message["say"].format(**self._memory)
        goal.response_handling = AudioOrchestrator.Goal.NO_RESPONSE
        
        await self._run_worker(self.__audio_orchestrator_client, goal, timeout_sec=self._orchestrator_timeout)
        
        self._next_message = None
    
    def _extract_message_based_on_retry(self) -> str:
        if not self._next_message:
            raise ValueError("next message not defined")
        if self._retry:
            if self._counter_retrys >= (MAX_TRY_SPEECH - 1):
                raise RetryError
            message_str = self._next_message["retry_say"]
            self._counter_retrys += 1
        else:
            message_str = self._next_message["say"]
            self._counter_retrys = 0
        
        self._retry = False
        return message_str
    
    def _determine_message_type(self) -> MESSAGE_TYPE:
        if self._next_message_name is not None and "action" in self._next_message_name:
            action = self._next_message_name["action"]
            match action:
                case "drive":
                    return MESSAGE_TYPE.DRIVE
                case "upload":
                    return MESSAGE_TYPE.UPLOAD
                case _:
                    raise ValueError(f"Action not defined: {action}")
        return MESSAGE_TYPE.STANDARD
    
    async def _react_to_standard_message(self): 
        message_str = self._extract_message_based_on_retry()
                   
        if not self._next_message:
            raise ValueError("Next message not defined")
        
        goal = self._assamble_orchestrator_goal(
            message_str, 
            AudioOrchestrator.Goal.TRANSCRIBE_RESPONSE, 
            transcribe_prompt=self._get_transcription_prompt())
        
        orchestrator_result = await self._run_worker(self.__audio_orchestrator_client, goal, timeout_sec=self._orchestrator_timeout)
        
        message = {
            "role": "user",
            "content": [{"text": orchestrator_result.transcribed_text}]
        }
        
        system_prompt = self._dialog_script["base_prompt"] + "\n" + self._next_message["prompt"]
        
        extract_list, transition_list, extract_id_options, tool = self._define_Tool(self._next_message)
        
        response = self._call_llm_api(system_prompt, [message], tool)
        
        reply = self._evaluate_llm_response(response, transition_list, extract_id_options)
        
        if reply["choice"] == "unclear":
            self._next_message = self._available_messages[self._next_message["fallback"]]
            self._retry = True
            return "continue"
        
        self._next_message_name = next((block["goto"] for block in self._next_message["transitions"] if block["if"] == reply["choice"]), None)
        self._next_message = self._available_messages[self._next_message_name] if self._next_message_name is not None else None
        
        for extraction in extract_list:
            self._memory[extraction] = reply[extraction]
            self.get_logger().info(f"Saved memory: {extraction}: {reply[extraction]}")
        
        return ""
    
    async def _react_to_drive_message(self):
        if not self._next_message:
            raise ValueError("Next message not defined")
        
        message = self._next_message["say"]
        
        worker_handle_audio = await self._send_goal(
            self.__audio_orchestrator_client, 
            self._assamble_orchestrator_goal(message, AudioOrchestrator.Goal.NO_RESPONSE)
        )
        
        if self._next_message["target"] != "professor":
            raise NotImplemented(f"No target with the name: {self._next_message["target"]} implemented")
        
        target = self._evaluate_hints_or_options(self._next_message["target"])
        
        target_coordinates = self._get_room_of_professor(target)
        self.get_logger().info(f"Coordinates: {target_coordinates}")
        
        self._away_from_station = True
        
        goal = MoveToLocation.Goal()
        goal.x = target_coordinates[0]
        goal.y = target_coordinates[1]
        goal.yaw = target_coordinates[2]
        
        try:
            result: MoveToLocation.Result = await self._run_worker(self.__driving_navigator_client, goal, timeout_sec=180.0)
        except GoalAbort:
            #todo get professor room
            self._next_message = self._available_messages[self._next_message["on_failure"]]
            return
        finally:
            await self._wait_for_result(worker_handle_audio, self._orchestrator_timeout)
        
        self._next_message = self._available_messages[self._next_message["on_success"]]
    
    async def _react_to_upload_message(self):
        if not self._next_message:
            raise ValueError("Next message not defined")
        
        message = self._next_message["say"]
        
        upload_para = UploadParams()
        upload_para.message = self._next_message["message"]
        upload_para.sender_name = self._evaluate_hints_or_options_string("name")
        upload_para.user_id = self._get_professor_id(self._evaluate_hints_or_options("professor"))
        
        try:
            await self._run_worker(
                self.__audio_orchestrator_client, 
                self._assamble_orchestrator_goal(message, AudioOrchestrator.Goal.UPLOAD_RESPONSE,upload_parameters=upload_para),
                timeout_sec=self._orchestrator_timeout
            )
        except GoalAbort:
            self._next_message = self._available_messages[self._next_message["on_failure"]]
        
        self._next_message = self._available_messages[self._next_message["on_success"]]
        
        
    def _evaluate_llm_response(self, response, transition_list, extract_id_options):
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
        
        for extraction_id in extract_id_options:
            if reply[extract_id_options] not in extract_id_options:
                raise ValueError(f"{reply[extraction_id]} not a valid answer option ({extract_id_options})")
        
        return reply
    
    def _get_transcription_prompt(self) -> str | None:
        transcribtion_prompt = None
        if self._next_message and "transcriber_hints" in self._next_message:
            try:
                transcribtion_prompt = "Das gesagte enthält: " + self._evaluate_hints_or_options_string(self._next_message["transcriber_hints"])   
            except Exception:
                self.get_logger().warning(f"Couldn't find {self._next_message["transcriber_hints"]} in memory")  
        return transcribtion_prompt
    
    def _assamble_orchestrator_goal(self, message: str, response_handling, transcribe_prompt: str | None = None, upload_parameters: UploadParams | None = None):
        goal = AudioOrchestrator.Goal()
        goal.question_or_sentence = message.format(**self._memory)
        goal.response_handling = response_handling
        if transcribe_prompt:
            goal.transcriber_parameters.initial_prompt = transcribe_prompt
        if upload_parameters:
            goal.upload_parameters = upload_parameters
        return goal
    
    def _get_professors(self) -> list[Any]:
        if "professors" in self._memory:
            return self._memory["professors"]
        
        #todo load professors via API
        #todo as json?
        professors = [{"Titel": "Prof. Dr.", "Vorname": "Bernd", "Nachname": "Lichte"}, {"Titel": "Prof. Dr.", "Vorname": "Michael", "Nachname": "Kolbus"}]
        self._memory["professors"] = professors
        return professors
    
    def _get_room_of_professor(self, professor):
        #todo API get the location of the professor
        return [1.0,2.0,180]
    
    def _get_professor_id(self, professor):
        #todo API implementation
        return 0
    
    def _evaluate_hints_or_options_string(self, hint_or_option: str):
        return_para = None
        hint_or_option_striped = hint_or_option.strip()
        match hint_or_option_striped:
            case "professors":
                return_para = []
                for professor in self._get_professors():
                    return_para.append(f"{professor.Titel} {professor.Vorname} {professor.Nachname}")
            case _:
                if hint_or_option_striped in self._memory:
                    return_para = self._memory[hint_or_option_striped]

        if isinstance(return_para, str):
            return return_para
        if isinstance(return_para, list):
            return ", ".join(return_para)
        raise ValueError(f"{hint_or_option_striped} not in memory or available via API")
    
    def _evaluate_hints_or_options(self, hint_or_option: str):
        hint_or_option_striped = hint_or_option.strip()
        match hint_or_option_striped.strip():
            case "professors":
                return self._get_professors()
            case _:
                if hint_or_option_striped in self._memory:
                    return self._memory[hint_or_option_striped]
                raise ValueError(f"{hint_or_option_striped} not in memory or available via API")
    
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
        extract_ids_options = []
        
        if "extract" in message:
            extract = message["extract"]
            for entry in extract:
                input_properties[entry] = {"type": "string", "description": extract[entry]["description"]}
                required.append(entry)
                extract_ids.append(entry)
                if "options" in entry:
                    options = self._evaluate_hints_or_options(entry["options"])
                    input_properties[entry]["options"] = options
                    extract_ids_options[entry] = (options)
                
        
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
        
        
        return (extract_ids, transitions_list, extract_ids_options,
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
            self._active_worker_handle.append(worker_handle)
        return worker_handle
    
    async def _wait_for_result(self, worker_handle: ClientGoalHandle, timeout_sec: float):
        try:
            response = await wait_for_universal_result(self, worker_handle, timeout_sec)
        finally:
            with self._lock_worker_handle:
                self._active_worker_handle.remove(worker_handle)

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
                    for worker_handle in self._active_worker_handle:
                        worker_handle.cancel_goal_async()
            return CancelResponse.ACCEPT 
    
    #todo On error say: "Tut mir leid ich habe momentan einen fehler"?
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