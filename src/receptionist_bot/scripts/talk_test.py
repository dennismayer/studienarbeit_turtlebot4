import yaml 

from pathlib import Path

import json
from typing import TYPE_CHECKING, Mapping, Any, Sequence
import time

import boto3
from botocore.config import Config
import rclpy
from rclpy.action.client import ActionClient
from rclpy.node import Node
from speaking_interfaces.action import AudioOrchestrator

from dotenv import load_dotenv

# type stubs are only needed for autocompletion/type checking, not at runtime
if TYPE_CHECKING:
    from mypy_boto3_bedrock_runtime.client import BedrockRuntimeClient
    from mypy_boto3_bedrock_runtime.type_defs import ConverseResponseTypeDef, ToolTypeDef

load_dotenv("/home/ubuntu/turtlebot4_ws/.env")

client: "BedrockRuntimeClient" = boto3.client(
    "bedrock-runtime",
    region_name="eu-central-1",
    config=Config(
        connect_timeout=5,
        read_timeout=15,                 # a short dialog reply takes ~1 s; don't let the robot hang for an hour
        retries={"max_attempts": 2},
    ))

client.close()

class OrchestratorClient(Node):
    """ROS 2 client for the Orchestrator action server."""

    def __init__(self, action_name: str = "speechRequest") -> None:
        super().__init__("orchestrator_client")
        self._action_client = ActionClient(self, AudioOrchestrator, action_name)

    def send_goal(self, goal: AudioOrchestrator.Goal):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Orchestrator action server is not available")
            return None

        return self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )

    def _feedback_callback(self, feedback_msg) -> None:
        self.get_logger().info(f"Orchestrator feedback: {feedback_msg.feedback}")

rclpy.init()
orchestrator_interface = OrchestratorClient()

def define_Tool(message: Mapping[str, Any]):
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

def call_llm_api(client_llm, system_prompt: str, messages: Sequence[Mapping[str, Any]], reply_tool):
    return client_llm.converse(
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

script_path = Path("/home/ubuntu/turtlebot4_ws/src/receptionist_bot") / "dialog_scripts" / "short_test_script.yaml"
with script_path.open("r", encoding="utf-8") as file:
	dialogue_script = yaml.safe_load(file)

print(type(dialogue_script))

print(define_Tool(dialogue_script["messages"]["ask_name"]))

retry = False
available_messages = dialogue_script["messages"]
next_message = available_messages[dialogue_script["start"]]
memory = {}

while next_message is not None:    
    if "end" in next_message and next_message["end"]:
        goal = AudioOrchestrator.Goal()
        goal.question_or_sentence = next_message["say"].format(**memory)
        goal.response_handling = AudioOrchestrator.Goal.NO_RESPONSE
        
        goal_future = orchestrator_interface.send_goal(goal)
        if goal_future is None:
            raise RuntimeError("Could not send goal to the Orchestrator")

        rclpy.spin_until_future_complete(orchestrator_interface, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Orchestrator goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(orchestrator_interface, result_future)
        
        next_message = None
    else:
        if retry:
            message_str = next_message["retry_say"]
        else:
            message_str = next_message["say"]
        
        retry = False
            
        goal = AudioOrchestrator.Goal()
        goal.question_or_sentence = message_str.format(**memory)
        goal.response_handling = AudioOrchestrator.Goal.TRANSCRIBE_RESPONSE
        
        goal_future = orchestrator_interface.send_goal(goal)
        if goal_future is None:
            raise RuntimeError("Could not send goal to the Orchestrator")

        rclpy.spin_until_future_complete(orchestrator_interface, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Orchestrator goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(orchestrator_interface, result_future)
        orchestrator_result = result_future.result().result
        response_text = orchestrator_result.transcribed_text
        
        message = {
            "role": "user",
            "content": [{"text": response_text}]
        }
        
        system_prompt = dialogue_script["base_prompt"] + "\n" + next_message["prompt"]
        
        extract_list, transition_list, tool = define_Tool(next_message)
        
        response = call_llm_api(client, system_prompt, [message], tool)
        
        # the structured answer arrives as an already parsed dict in the toolUse block, not as text
        tool_use = next((block["toolUse"] for block in response["output"]["message"]["content"] if "toolUse" in block), None)
        if response["stopReason"] == "tool_use" and tool_use is not None:
            reply = tool_use["input"]
        else:
            raise ValueError(f"No valid response: {response["output"]["message"]}")
        
        print(reply)
        
        if reply["choice"] not in transition_list:
            raise ValueError(f"{reply["choice"]} not a valid answer option ({transition_list})")
        
        if reply["choice"] == "unclear":
            retry = True
            continue
        
        next_message_name = next((block["goto"] for block in next_message["transitions"] if block["if"] == reply["choice"]), None)
        next_message = available_messages[next_message_name] if next_message_name is not None else None
        
        for extraction in extract_list:
            memory[extraction] = reply[extraction]
            print(f"Saved memory: {extraction}: {reply[extraction]}")
