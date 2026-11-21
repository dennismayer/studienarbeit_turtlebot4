import requests
import sys
import os
from dotenv import load_dotenv
from datetime import datetime
from zoneinfo import ZoneInfo
from typing import Any
from pathlib import Path

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from server_interfaces.msg import Professor, RoomCoordinates
from server_interfaces.srv import GetConsultationHour, GetProfessors
from server_interfaces.action import UploadFile

from robot_common.parameter_helper import get_param

DEFAULT_NODE_NAME = "ServerConnectorNode"

class ConnectorServer(Node):
    def __init__(self):
        super().__init__(DEFAULT_NODE_NAME)
        self._declare_node_parameters()
        self._declare_node_variables()
        self._add_services()
        self._start_action_server()

    def _declare_node_parameters(self) -> None:
        self.declare_parameters('', [
            ("get_professors_service", "get_professors",
                ParameterDescriptor(
                    description="Name of the service that returns the list of professors",
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("get_consultation_hour_service", "get_consultation_hour",
                ParameterDescriptor(
                    description="Name of the service that returns the next consultation hour of a professor",
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("upload_file_action", "upload_file",
                ParameterDescriptor(
                    description="Name of the action server that uploads messages and audio files",
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("web_address", "http://192.168.0.92:8080",
                ParameterDescriptor(
                    description="Base URL of the web server",
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("env_file_path", "/home/ubuntu/turtlebot4_ws/.env",
                ParameterDescriptor(
                    description="Path of the .env file, only loaded when ENVIRONMENT=dev",
                    read_only=True,
                    type=ParameterType.PARAMETER_STRING)),
            ("server_timeout", 5.0,
                ParameterDescriptor(
                    description="Timeout in seconds for each HTTP request to the web server and S3",
                    read_only=True,
                    type=ParameterType.PARAMETER_DOUBLE)),
        ])

    def _declare_node_variables(self) -> None:
        env_file_path = get_param(self, "env_file_path", str)
        if os.getenv("ENVIRONMENT") != "prod":
            if not load_dotenv(env_file_path):
                raise FileNotFoundError(f"No .env file at: {env_file_path}")
            self.get_logger().debug(f"Loaded environment from {env_file_path}")

        self._webserver = get_param(self, "web_address", str)
        self._server_timeout = get_param(self, "server_timeout", float)
        self.get_logger().info(f"Using web server {self._webserver} (request timeout {self._server_timeout:.1f} s)")
        self._api_token = os.getenv("SERVER_API_TOKEN")
        if not self._api_token:
            raise ValueError("No API Token set!")
        
        self._standard_header = {"Authorization": f"Bearer {self._api_token}"}
        
        self._check_if_server_is_reachable()        
    
    def _add_services(self):
        get_professors_service = get_param(self, "get_professors_service", str)
        self._get_professors_server = self.create_service(
            GetProfessors,
            get_professors_service,
            self._callback_get_professors,
            callback_group=ReentrantCallbackGroup()
        )
        get_consultation_hour_service = get_param(self, "get_consultation_hour_service", str)
        self._get_consultation_hour = self.create_service(
            GetConsultationHour,
            get_consultation_hour_service,
            self._callback_get_consultation_hour,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info(f"Services '{get_professors_service}' and '{get_consultation_hour_service}' ready")

    def _start_action_server(self):
        upload_file_action = get_param(self, "upload_file_action", str)
        self._upload_file_action = ActionServer(
            self,
            UploadFile,
            upload_file_action,
            self._execute_upload_file_action,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info(f"Action server '{upload_file_action}' ready, waiting for requests")

    def _callback_get_professors(self, _, response):
        self.get_logger().info("Professor list requested")
        try:
            response_api: requests.Response = requests.get(
                self._webserver + "/api/get_professors",
                headers=self._standard_header,
                timeout=self._server_timeout
            )
            
            if response_api.status_code != 200:
                raise ValueError(f"Received wrong status code in get_professors: {response_api.status_code}")
            
            response_content = response_api.json()
            professors = []
            
            for professor in response_content:
                prof = Professor()       
                prof.id = int(professor["user_id"])
                prof.title = professor["academic_title"] if professor["academic_title"] else ""
                prof.first_name = professor["firstname"] if professor["firstname"] else ""
                prof.last_name = professor["surname"] if professor["surname"] else ""
                professors.append(prof)

            self.get_logger().info(f"Returning {len(professors)} professor(s)")
            response.success = True
            response.professors = professors
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error while creating professor list: {e}")
            response.success = False
            response.professors = []
            return response
    
    def _callback_get_consultation_hour(self, request, response):
        self.get_logger().info(f"Consultation hour requested for professor {request.prof_id}")
        try:
            response_api: requests.Response = requests.get(
                self._webserver + "/api/slot_data/get_next_timeslot_info",
                params={"id": request.prof_id},
                headers=self._standard_header,
                timeout=self._server_timeout
            )
            
            match response_api.status_code:
                case 204:
                    # no current nor future consultation hours
                    self.get_logger().info(f"Professor {request.prof_id} has no upcoming consultation hour")
                    return self._assemble_get_consultation_hour(
                        response, 
                        GetConsultationHour.Response.NO_CONSULTATION_HOUR
                    )
                case 200:
                    # successful data recovery
                    response_content = response_api.json()
                    self.get_logger().info(
                        f"Professor {request.prof_id}: room '{response_content['room_name']}', "
                        f"next {response_content['next_occurrence']} from {response_content['from']} "
                        f"(now: {response_content['next_occurrence_now']})"
                    )
                    self.get_logger().debug(
                        f"Room coordinates x={response_content['x']}, y={response_content['y']}, yaw={response_content['yaw']}"
                    )
                    return self._assemble_get_consultation_hour(
                        response,
                        GetConsultationHour.Response.SUCCESS,
                        response_content["next_occurrence_now"],
                        response_content["room_name"],
                        self._get_date_string(response_content["next_occurrence"], response_content["from"]),
                        response_content["x"],
                        response_content["y"],
                        response_content["yaw"]
                    )    
                case _:
                    raise ValueError(f"Unknown response code: {response_api.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error while getting consultation hour: {e}")
            return self._assemble_get_consultation_hour(
                response,
                GetConsultationHour.Response.ERROR
            )
    
    def _get_date_string(self, next_occurrence, time_from):
        dt = datetime.fromisoformat(next_occurrence).astimezone(ZoneInfo('Europe/Berlin'))
        return f"{dt.day}.{dt.month}.{dt.year};{time_from[:5]}"
                            
    
    def _assemble_get_consultation_hour(
            self, 
            response, 
            response_code: int,
            current_hour: bool | None=None, 
            room_name: str | None=None, 
            next_consultation_hour: str | None=None, 
            x: Any | None=None, 
            y: Any | None=None, 
            yaw: Any | None=None
            ):
        response.return_code = response_code
        response.current_consultation_hour = current_hour if current_hour else False
        response.room_name = room_name if room_name else ""
        response.next_consultation_hour = next_consultation_hour if next_consultation_hour else ""
        room_coordinates = RoomCoordinates()
        room_coordinates.x = float(x) if x else 0.0
        room_coordinates.y = float(y) if y else 0.0
        room_coordinates.yaw = float(yaw) if yaw else 0.0
        response.room_coordinates = room_coordinates
        return response
        
    def _execute_upload_file_action(self, goal_handle: ServerGoalHandle) -> UploadFile.Result:
        try:
            self._send_feedback(goal_handle, UploadFile.Feedback.RECEIVED)
            
            request: UploadFile.Goal = goal_handle.request
            self.get_logger().info(
                f"Upload requested for user {request.user_id} from '{request.sender_name}' (send_file: {request.send_file})"
            )

            # pre check
            file_path = request.file_path
            if request.send_file and (not Path(file_path).is_file() or not file_path.endswith(".wav")):
                raise FileNotFoundError(f"Couldn't find file at: {file_path}")
            
            self.get_logger().debug(f"Posting message to {self._webserver}/api/messages")
            message = {
                "user_id": request.user_id,
                "sender_name": request.sender_name,
                "message": request.message,
                "audio_duration_s": request.audio_duration_s,
                "send_file": request.send_file
            }
            
            api_response = requests.post(
                self._webserver + "/api/messages",
                json=message,
                headers=self._standard_header,
                timeout=self._server_timeout
                )
            
            if not api_response.ok:
                raise ValueError(f"Received wrong status code: {api_response.status_code}")

            response_content = {}
            
            if api_response.status_code == 200:
                response_content = api_response.json()

            self.get_logger().info(f"Message created on server (status {api_response.status_code})")

            if "url" in response_content and response_content["url"]:
                self._send_feedback(goal_handle, UploadFile.Feedback.UPLOAD_LINK_RECEIVED)
                self.get_logger().info(
                    f"Upload link received, uploading {Path(file_path).name} ({Path(file_path).stat().st_size} bytes)..."
                )
                with open(Path(file_path), mode="rb") as file:
                    s3_response = requests.put(
                        response_content["url"],
                        data=file,
                        headers={"Content-Type": "audio/wav"},
                        timeout=self._server_timeout
                    )
                
                if not s3_response.ok:
                    raise ValueError(f"Received wrong status code during s3 upload: {s3_response.status_code}")
                
                self._send_feedback(goal_handle, UploadFile.Feedback.UPLOADED)
            else:
                if request.send_file:
                    raise ValueError("File should be sent but wasn't")

                self.get_logger().info("Upload finished (message only, no file)")
                goal_handle.succeed()
                return self._assemble_upload_result(True, "UploadWithoutFile")

            self.get_logger().info("Upload finished (message and file)")
            goal_handle.succeed()
            return self._assemble_upload_result(True, "UploadWithFile")
            
        except Exception as e:
            self.get_logger().error(f"Error while trying to upload a file: {e}")
            goal_handle.abort()
            return self._assemble_upload_result(False, f"FailedWithError: {e}")
    
    def _assemble_upload_result(self, success: bool = False, message: str=""):
        result = UploadFile.Result()
        result.success = success
        result.message = message
        return result
    
    def _send_feedback(self, goal_handle: ServerGoalHandle, state):
        feedback = UploadFile.Feedback()
        feedback.state = state
        goal_handle.publish_feedback(feedback)
    
    def _check_if_server_is_reachable(self):
        try:
            response: requests.Response = requests.get(
                self._webserver + "/api/online", 
                headers=self._standard_header,
                timeout=self._server_timeout
            )
            if response.status_code == 200:
                self.get_logger().info(f"Server {self._webserver} is online!")
            else:
                raise ConnectionError(f"Connection failed with Code: {response.status_code}")
        except Exception as e:
            error_string = f"Server {self._webserver} not reachable: {e}"
            self.get_logger().error(error_string)
            raise ConnectionError(error_string) from e

def main(args=None):
    rclpy.init(args=args)

    connector_server = None
    executor = None
    exit_code = 0

    try:
        connector_server = ConnectorServer()
        executor = MultiThreadedExecutor()
        executor.add_node(connector_server)
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
        if connector_server is not None:
            connector_server.destroy_node()
        try_shutdown()

    return exit_code

if __name__ == "__main__":
    sys.exit(main())
