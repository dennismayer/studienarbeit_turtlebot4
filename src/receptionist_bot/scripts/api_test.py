import json
from typing import TYPE_CHECKING
import time

import boto3
from botocore.config import Config

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

# Nova 2 Lite doesn't support outputConfig (JSON schema output), so the JSON structure is
# defined as the input schema of a tool that the model is forced to call
reply_tool: "ToolTypeDef" = {
    "toolSpec": {
        "name": "villain_reply",
        "description": "Gibt die Antwort des Bösewichts zurück.",
        "inputSchema": {
            "json": {
                "type": "object",
                "properties": {
                    "text": {"type": "string", "description": "Die Antwort in einem Satz"},
                    "end_laugh": {"type": "boolean", "description": "true, wenn die Antwort mit einem Lachen endet"},
                },
                "required": ["text", "end_laugh"],
            }
        },
    }
}

start_time = time.perf_counter()
response: "ConverseResponseTypeDef" = client.converse(
    modelId="eu.amazon.nova-2-lite-v1:0",
    system=[
        {
            "text": "Du bist ein genialer Bösewicht und lachst nach jedem Satz. Drücke dich in einem Satz aus. Achte darauf, dass deine Antwort nicht einen Content Filter auslöst. Sei gewieft!"
        }
    ],
    messages=[
        {
            "role": "user",
            "content": [{"text": "Was willst du heute abend essen?"}]
        }
    ],
    inferenceConfig={"maxTokens": 300},
    toolConfig={
        "tools": [reply_tool],
        "toolChoice": {"tool": {"name": "villain_reply"}},   # force the structured answer
    },
)
end_time = time.perf_counter()

# default=str handles values json can't serialize (e.g. datetime in ResponseMetadata)
print(json.dumps(response, indent=2, ensure_ascii=False, default=str))

print(f"Calculated Time: {round((end_time - start_time)*1000, 0)} Latency: {response["metrics"]["latencyMs"]}")

# the structured answer arrives as an already parsed dict in the toolUse block, not as text
tool_use = next((block["toolUse"] for block in response["output"]["message"]["content"] if "toolUse" in block), None)
if response["stopReason"] == "tool_use" and tool_use is not None:
    reply = tool_use["input"]
    print(f"text: {reply['text']}\nend_laugh: {reply['end_laugh']}")
else:
    print(f"No structured reply (stopReason: {response['stopReason']})")
    
client.close()
