#!/usr/bin/env python3
"""
Car Museum Guide Robot
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String, Bool
from openai import AzureOpenAI
import json
import time
from enum import Enum
from threading import Lock, Timer
import os
from dotenv import load_dotenv
from pathlib import Path
import subprocess
import sys
import select
from ament_index_python.packages import get_package_share_directory

# Load environment variables
env_path = Path(__file__).resolve()
for parent in env_path.parents:
    env_file = parent / '.env'
    if env_file.exists():
        load_dotenv(env_file, override=False)
        break


class RobotState(Enum):
    """Robot operational states"""
    STANDBY = 1
    NAVIGATING = 2
    PRESENTING = 3
    QA_MODE = 4
    WAITING_FOR_COMMAND = 5


class CarMuseumGuideBot(Node):
    """Main museum guide robot controller"""
    
    def __init__(self):
        super().__init__('car_museum_guide')

        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter('venv_python_path', '')
        self.declare_parameter('tts_worker_script', '')
        self.declare_parameter('azure_openai_endpoint', os.getenv('AZURE_OPENAI_ENDPOINT', ''))
        self.declare_parameter('azure_openai_key', os.getenv('AZURE_OPENAI_API_KEY', ''))
        self.declare_parameter('azure_openai_deployment', os.getenv('AZURE_OPENAI_DEPLOYMENT_NAME', 'gpt-4'))
        self.declare_parameter('exhibits_config', 'car_exhibits.json')
        self.declare_parameter('enable_visitor_detection', True)
        self.declare_parameter('home_position_x', 0.0)
        self.declare_parameter('home_position_y', 0.0)
        self.declare_parameter('home_position_theta', 0.0)
        self.declare_parameter('auto_return_home', True)
        self.declare_parameter('idle_timeout', 300.0)
        self.declare_parameter('tts_timeout', 20.0)
        self.declare_parameter('nav_retry_attempts', 3)
        self.declare_parameter('nav_retry_delay', 5.0)

        # Get parameters
        self.venv_python = self.get_parameter('venv_python_path').value
        tts_worker_script = self.get_parameter('tts_worker_script').value
        self.tts_timeout = self.get_parameter('tts_timeout').value
        self.nav_retry_attempts = self.get_parameter('nav_retry_attempts').value
        self.nav_retry_delay = self.get_parameter('nav_retry_delay').value

        # Validate required parameters
        if not self.venv_python:
            self.get_logger().error('Required parameter "venv_python_path" not set')
            raise ValueError('Missing required parameter: venv_python_path')

        if not tts_worker_script:
            self.get_logger().error('Required parameter "tts_worker_script" not set')
            raise ValueError('Missing required parameter: tts_worker_script')

        if not os.path.exists(self.venv_python):
            raise FileNotFoundError(f'Python executable not found: {self.venv_python}')

        if not os.path.exists(tts_worker_script):
            raise FileNotFoundError(f'TTS worker script not found: {tts_worker_script}')

        self.tts_worker_script = tts_worker_script

        # Get other parameters
        self.azure_openai_endpoint = self.get_parameter('azure_openai_endpoint').value
        self.azure_openai_key = self.get_parameter('azure_openai_key').value
        self.openai_deployment = self.get_parameter('azure_openai_deployment').value
        exhibits_filename = self.get_parameter('exhibits_config').value
        self.enable_visitor_detection = self.get_parameter('enable_visitor_detection').value

        # Home position
        self.home_x = self.get_parameter('home_position_x').value
        self.home_y = self.get_parameter('home_position_y').value
        self.home_theta = self.get_parameter('home_position_theta').value
        self.auto_return_home = self.get_parameter('auto_return_home').value
        self.idle_timeout = self.get_parameter('idle_timeout').value

        # Validate Azure OpenAI credentials
        if not self.azure_openai_endpoint:
            self.get_logger().error('AZURE_OPENAI_ENDPOINT not set')
            raise ValueError('Missing Azure OpenAI endpoint')

        if not self.azure_openai_key:
            self.get_logger().error('AZURE_OPENAI_API_KEY not set')
            raise ValueError('Missing Azure OpenAI API key')

        if not self.openai_deployment:
            self.get_logger().error('AZURE_OPENAI_DEPLOYMENT_NAME not set')
            raise ValueError('Missing Azure OpenAI deployment name')

        self.get_logger().info(f'Azure OpenAI Endpoint: {self.azure_openai_endpoint}')
        self.get_logger().info(f'Azure OpenAI Deployment: {self.openai_deployment}')

        # Resolve exhibit config path
        try:
            pkg_share = get_package_share_directory('car_museum_guide')
            exhibits_file = os.path.join(pkg_share, exhibits_filename)
        except Exception as e:
            self.get_logger().warn(f'Could not find package share directory: {e}')
            exhibits_file = exhibits_filename

        # Initialize TTS Worker Process
        self.tts_process = None
        self.get_logger().info('Starting TTS Worker (gTTS)')
        self.start_tts_worker()

        # Initialize Azure OpenAI
        api_version = os.getenv('API_VERSION', '2024-08-01-preview')

        # Test connection
        self.get_logger().info('Testing Azure OpenAI connection')
        try:
            self.openai_client = AzureOpenAI(
                azure_endpoint=self.azure_openai_endpoint,
                api_key=self.azure_openai_key,
                api_version=api_version
            )

            # Test with minimal request
            test_response = self.openai_client.chat.completions.create(
                model=self.openai_deployment,
                messages=[{"role": "user", "content": "Test"}],
                max_completion_tokens=50000
            )

            test_content = test_response.choices[0].message.content
            if not test_content or len(test_content.strip()) == 0:
                self.get_logger().warn('Model returns empty content - using adaptive fallbacks')

            self.get_logger().info('Azure OpenAI connected successfully')

        except Exception as e:
            self.get_logger().error(f'Azure OpenAI connection failed: {e}')
            raise

        # Load exhibits configuration
        self.exhibits = self.load_exhibits(exhibits_file)

        # Cache for AI-generated content
        self.exhibit_descriptions = {}
        self.conversation_history = []

        # Initialize BasicNavigator
        self.get_logger().info('Initializing BasicNavigator')
        self.nav = BasicNavigator()
        
        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2 (30s timeout)')
        self.nav.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready')

        # Subscribers
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback,
            10, callback_group=self.callback_group
        )

        self.wake_word_sub = self.create_subscription(
            Bool, 'wake_word_detected', self.wake_word_callback,
            10, callback_group=self.callback_group
        )

        self.visitor_sub = self.create_subscription(
            String, 'visitor_detected', self.visitor_callback,
            10, callback_group=self.callback_group
        )

        # Publisher for status
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # State management
        self.current_state = RobotState.STANDBY
        self.current_tour = []
        self.current_exhibit_index = 0
        self.current_exhibit_name = None
        self.qa_mode_active = False
        self.visitor_present = False

        # Thread safety
        self.state_lock = Lock()

        # Timers
        self.idle_timer = None
        self.idle_timer_lock = Lock()

        # Health check for TTS worker
        self.tts_health_timer = self.create_timer(10.0, self.check_tts_health)

        self.get_logger().info('Car Museum Guide initialized successfully')
        self.get_logger().info(f'Loaded {len(self.exhibits)} exhibits')
        self.get_logger().info(f'TTS timeout: {self.tts_timeout}s')
        self.get_logger().info(f'Navigation retry: {self.nav_retry_attempts}x')

        # Generate initial descriptions
        self.get_logger().info('Generating exhibit descriptions')
        self.generate_all_descriptions()

        if self.enable_visitor_detection:
            self.get_logger().info('Visitor detection enabled')
        else:
            self.speak("Hallo! Ich bin Ihr Automuseum-Führer. Sagen Sie Hey Computer.")

    def start_tts_worker(self):
        """Start the TTS worker subprocess"""
        try:
            env = os.environ.copy()

            self.tts_process = subprocess.Popen(
                [self.venv_python, self.tts_worker_script],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=sys.stderr,
                text=True,
                bufsize=1,
                env=env
            )

            ready = select.select([self.tts_process.stdout], [], [], 15.0)
            if not ready[0]:
                raise RuntimeError('TTS Worker timeout waiting for READY signal')

            ready_signal = self.tts_process.stdout.readline().strip()

            if ready_signal == "READY":
                self.get_logger().info('TTS Worker ready')
            else:
                raise RuntimeError(f'TTS Worker failed to initialize: {ready_signal}')

        except Exception as e:
            self.get_logger().error(f'Failed to start TTS subprocess: {e}')
            raise

    def check_tts_health(self):
        """Periodic health check of TTS worker with auto-restart"""
        if self.tts_process and self.tts_process.poll() is not None:
            self.get_logger().error('TTS worker process died - attempting restart')
            try:
                self.start_tts_worker()
                self.get_logger().info('TTS worker restarted successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to restart TTS worker: {e}')

    def load_exhibits(self, filename):
        """Load exhibit information from JSON file"""
        try:
            with open(filename, 'r') as f:
                exhibits = json.load(f)
            self.get_logger().info(f'Loaded {len(exhibits)} exhibits from {filename}')
            return exhibits
        except Exception as e:
            self.get_logger().error(f'Failed to load exhibits: {str(e)}')
            return {}

    def sanitize_text(self, text):
        """Remove problematic characters that break TTS"""
        if not text:
            return ""

        text = text.replace('"', '').replace('"', '').replace('"', '')
        text = text.replace('«', '').replace('»', '')
        text = text.replace(''', '').replace(''', '')
        text = text.replace("'", '')
        text = text.replace('`', '').replace('~', '')

        return text.strip()

    def truncate_to_sentence(self, text, max_chars=200):
        """Intelligently truncate to first complete sentence"""
        if not text:
            return ""

        text = text.strip()

        if '.' in text:
            first_period = text.find('.')
            if first_period > 0 and first_period < max_chars:
                return text[:first_period + 1]

        if len(text) > max_chars:
            truncate_pos = text[:max_chars - 5].rfind(' ')
            if truncate_pos > 40:
                return text[:truncate_pos] + '.'
            else:
                return text[:max_chars - 5] + '.'

        if not text.endswith('.'):
            text += '.'

        return text

    def generate_all_descriptions(self):
        """Pre-generate descriptions for all exhibits"""
        for exhibit_name in self.exhibits.keys():
            try:
                description = self.generate_exhibit_description(exhibit_name)
                self.exhibit_descriptions[exhibit_name] = description
                self.get_logger().info(f'Generated description for {exhibit_name}')
            except Exception as e:
                self.get_logger().error(f'Failed to generate description for {exhibit_name}: {e}')
                fallback = f"Der {exhibit_name} ist ein klassisches Automobil."
                self.exhibit_descriptions[exhibit_name] = fallback

    def generate_exhibit_description(self, exhibit_name):
        """Generate engaging description using Azure OpenAI"""
        prompt = f"Beschreibe den {exhibit_name} in 1-2 kurzen Sätzen. Maximal 30 Wörter. Nur Deutsch."

        try:
            response = self.openai_client.chat.completions.create(
                model=self.openai_deployment,
                messages=[
                    {"role": "system", "content": "Du antwortest in 1-2 kurzen Sätzen (max 30 Wörter). Immer mit Punkt am Ende."},
                    {"role": "user", "content": prompt}
                ],
                max_completion_tokens=50000
            )

            description = response.choices[0].message.content

            if not description or not description.strip():
                return f"Der {exhibit_name} ist ein faszinierendes klassisches Auto."

            description = self.sanitize_text(description)
            description = self.truncate_to_sentence(description, max_chars=200)

            return description

        except Exception as e:
            self.get_logger().error(f'OpenAI API error for {exhibit_name}: {str(e)}')
            return f"Der {exhibit_name} ist ein klassisches Automobil."

    def speak(self, text):
        """Send text to TTS worker with robust error handling and auto-recovery"""
        if not text:
            return

        # Check if TTS process is alive, restart if needed
        if not self.tts_process or self.tts_process.poll() is not None:
            self.get_logger().error("TTS process is dead - attempting restart")
            try:
                self.start_tts_worker()
            except Exception as e:
                self.get_logger().error(f"Failed to restart TTS: {e}")
                return

        try:
            text = self.sanitize_text(text)

            if not text:
                return

            self.get_logger().info(f'Speaking ({len(text)} chars): {text[:60]}...')

            self.tts_process.stdin.write(text + "\n")
            self.tts_process.stdin.flush()

            ready = select.select([self.tts_process.stdout], [], [], self.tts_timeout)

            if not ready[0]:
                self.get_logger().error(f"TTS timeout after {self.tts_timeout:.0f}s")
                return

            response = self.tts_process.stdout.readline().strip()

            if response == "DONE":
                self.get_logger().info("TTS complete")
            else:
                self.get_logger().warning(f"Unexpected TTS response: {response}")

        except BrokenPipeError:
            self.get_logger().error("TTS pipe broken - restarting worker")
            try:
                self.start_tts_worker()
            except Exception as e:
                self.get_logger().error(f"Failed to restart TTS: {e}")
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

    def visitor_callback(self, msg):
        """Handle visitor detection events"""
        event = msg.data

        if event == "visitor_approaching":
            with self.state_lock:
                if not self.visitor_present and self.current_state == RobotState.STANDBY:
                    self.visitor_present = True
                    self.get_logger().info('Visitor detected - greeting')

            self.cancel_idle_timer()
            self.speak("Hallo! Ich bin Ihr Automuseum-Führer. Sagen Sie Hey Computer.")

        elif event == "visitor_left":
            with self.state_lock:
                self.visitor_present = False
                self.get_logger().info('Visitor left')

            if self.auto_return_home:
                self.reset_idle_timer()

    def wake_word_callback(self, msg):
        """Handle wake word detection"""
        if msg.data:
            self.get_logger().info('Wake word detected - ready for command')

    def _normalize_command(self, command_text):
        """Normalize common speech recognition errors"""
        replacements = {
            'seit mir': 'zeig mir',
            'zeit mir': 'zeig mir',
            'seid mir': 'zeig mir',
            'seite mir': 'zeig mir',
            'seig mir': 'zeig mir',
            'sait mir': 'zeig mir',
            'feig mir': 'zeig mir'
        }
        
        for wrong, correct in replacements.items():
            command_text = command_text.replace(wrong, correct)
        
        return command_text

    def command_callback(self, msg):
        """Process voice commands with intelligent intent detection"""
        command_text = self._normalize_command(msg.data.lower())
        self.get_logger().info(f'Received command: {command_text}')

        self.cancel_idle_timer()

        with self.state_lock:
            current_state = self.current_state
            qa_mode = self.qa_mode_active

        if any(word in command_text for word in ['abbrechen', 'stopp', 'stop', 'beenden', 'egal']):
            self.cancel_tour()
            return

        exhibit_name = self.parse_exhibit_name(command_text)

        is_navigation_command = any(word in command_text for word in [
            'bring mich', 'zeig mir', 'geh zu', 'navigiere', 'fahre zu',
            'führe mich', 'zeige', 'bring', 'gehe', 'götst', 'kannst du'
        ])

        if qa_mode and not is_navigation_command:
            self.answer_question(command_text)
            return

        if exhibit_name and is_navigation_command:
            self.start_tour([exhibit_name])
            return

        if exhibit_name:
            self.start_tour([exhibit_name])
            return

        self.get_logger().info('No exhibit found - treating as general question')
        self.answer_general_question(command_text)

    def parse_exhibit_name(self, command_text):
        """Extract exhibit name from command"""
        for exhibit_name in self.exhibits.keys():
            if exhibit_name.lower() in command_text:
                return exhibit_name

        for exhibit_name in self.exhibits.keys():
            name_parts = exhibit_name.lower().split()
            if any(part in command_text for part in name_parts if len(part) > 3):
                return exhibit_name

        return None

    def answer_general_question(self, question):
        """Answer general questions using Azure OpenAI"""
        self.get_logger().info(f'Answering general question: {question}')

        museum_context = f"""Du bist Automuseum-Führer. Museum hat {len(self.exhibits)} Autos.
WICHTIG: Antworte in 1-2 Sätzen (max 30 Wörter). Punkt am Ende. Nur Deutsch."""

        try:
            self.get_logger().info('Calling Azure OpenAI API')

            response = self.openai_client.chat.completions.create(
                model=self.openai_deployment,
                messages=[
                    {"role": "system", "content": museum_context},
                    {"role": "user", "content": question}
                ],
                max_completion_tokens=50000
            )

            answer = response.choices[0].message.content

            if not answer or not answer.strip():
                self.get_logger().error('Empty response from OpenAI API')

                if any(word in question.lower() for word in ['wie viele', 'wie viel', 'anzahl']):
                    answer = f"Wir haben {len(self.exhibits)} klassische Autos."
                elif any(word in question.lower() for word in ['welche', 'namen', 'was für']):
                    answer = "Verschiedene klassische Automobile."
                else:
                    answer = "Möchten Sie ein bestimmtes Auto sehen?"
            else:
                answer = self.sanitize_text(answer.strip())
                answer = self.truncate_to_sentence(answer, max_chars=200)

            self.get_logger().info(f'Got answer ({len(answer)} chars): {answer}')
            self.speak(answer)

        except Exception as e:
            self.get_logger().error(f'Error generating answer: {e}')
            self.speak("Entschuldigung, ich konnte Ihre Frage nicht beantworten.")

    def answer_question(self, question):
        """Answer visitor's question about current exhibit using Azure OpenAI"""
        self.get_logger().info(f'Answering exhibit question: {question}')

        with self.state_lock:
            if not self.current_tour or self.current_exhibit_index == 0:
                exhibit_name = "dem aktuellen Exponat"
                exhibit_description = ""
            else:
                exhibit_name = self.current_tour[self.current_exhibit_index - 1]
                exhibit_description = self.exhibit_descriptions.get(exhibit_name, "")

        self.conversation_history.append({"role": "user", "content": question})

        system_prompt = f"""Automuseum-Führer beim {exhibit_name}.
Kontext: {exhibit_description}
Antworte in 1-2 kurzen Sätzen (max 30 Wörter). Punkt am Ende. Nur Deutsch."""

        messages = [{"role": "system", "content": system_prompt}]
        messages.extend(self.conversation_history[-4:])

        try:
            self.get_logger().info('Calling Azure OpenAI API for exhibit question')

            response = self.openai_client.chat.completions.create(
                model=self.openai_deployment,
                messages=messages,
                max_completion_tokens=50000
            )

            answer = response.choices[0].message.content

            if not answer or not answer.strip():
                self.get_logger().error('Empty response for exhibit question')
                answer = "Haben Sie noch andere Fragen?"
            else:
                answer = self.sanitize_text(answer.strip())
                answer = self.truncate_to_sentence(answer, max_chars=200)

            self.conversation_history.append({"role": "assistant", "content": answer})
            self.get_logger().info(f'Got answer ({len(answer)} chars): {answer}')
            self.speak(answer)

        except Exception as e:
            self.get_logger().error(f'Error generating answer: {e}')
            self.speak("Haben Sie noch andere Fragen?")

    def start_tour(self, exhibit_list):
        """Start a tour"""
        with self.state_lock:
            if self.current_state == RobotState.NAVIGATING:
                self.speak("Ich navigiere gerade. Bitte warten Sie.")
                return

            self.current_tour = exhibit_list
            self.current_exhibit_index = 0
            self.qa_mode_active = False
            self.conversation_history = []

        self.navigate_to_next_exhibit()

    def create_pose(self, x, y, orientation):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        from math import sin, cos
        pose.pose.orientation.z = sin(float(orientation) / 2.0)
        pose.pose.orientation.w = cos(float(orientation) / 2.0)

        return pose

    def navigate_to_next_exhibit(self):
        """Navigate to next exhibit with retry logic"""
        with self.state_lock:
            if self.current_exhibit_index >= len(self.current_tour):
                self.current_state = RobotState.STANDBY
                self.speak("Tour beendet! Möchten Sie noch etwas anderes sehen?")
                return

            exhibit_name = self.current_tour[self.current_exhibit_index]

            if exhibit_name not in self.exhibits:
                self.speak(f"Entschuldigung, ich habe keine Informationen über {exhibit_name}.")
                self.current_state = RobotState.STANDBY
                return

            exhibit_data = self.exhibits[exhibit_name]
            self.current_exhibit_name = exhibit_name
            self.current_state = RobotState.NAVIGATING

        self.speak(f"Ich bringe Sie zum {exhibit_name}.")

        # Extract position
        if 'position' in exhibit_data:
            pos = exhibit_data['position']
            x = pos['x']
            y = pos['y']
            orientation = pos.get('orientation', pos.get('theta', 0.0))
        else:
            x = exhibit_data['x']
            y = exhibit_data['y']
            orientation = exhibit_data.get('orientation', exhibit_data.get('theta', 0.0))

        goal_pose = self.create_pose(x, y, orientation)

        # Navigation with retry logic
        for attempt in range(1, self.nav_retry_attempts + 1):
            if attempt > 1:
                self.get_logger().info(f'Retry {attempt}/{self.nav_retry_attempts} for {exhibit_name}')
                time.sleep(self.nav_retry_delay)

            self.get_logger().info(f'Navigating to {exhibit_name} (Attempt {attempt}/{self.nav_retry_attempts})')

            # Send goal
            self.nav.goToPose(goal_pose)

            # Wait for completion
            while not self.nav.isTaskComplete():
                try:
                    feedback = self.nav.getFeedback()
                    if feedback and hasattr(feedback, 'distance_remaining'):
                        self.get_logger().info(
                            f'Distance remaining: {feedback.distance_remaining:.2f}m',
                            throttle_duration_sec=2.0
                        )
                except Exception as e:
                    self.get_logger().warn(f'Feedback error: {e}')

                time.sleep(0.1)

            # Check result
            result = self.nav.getResult()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Reached {exhibit_name}')
                with self.state_lock:
                    self.current_state = RobotState.PRESENTING
                time.sleep(1)
                self.present_exhibit()
                return

            elif result == TaskResult.CANCELED:
                self.get_logger().warning(f'Navigation canceled at {exhibit_name} (Attempt {attempt})')

            elif result == TaskResult.FAILED:
                self.get_logger().warning(f'Failed to reach {exhibit_name} (Attempt {attempt})')

            # Last attempt failed
            if attempt == self.nav_retry_attempts:
                self.get_logger().error(f'Could not reach {exhibit_name} after {self.nav_retry_attempts} attempts')
                self.speak("Entschuldigung, ich hatte Schwierigkeiten, dieses Auto zu erreichen.")
                with self.state_lock:
                    self.current_state = RobotState.STANDBY
                return

    def present_exhibit(self):
        """Present exhibit information"""
        with self.state_lock:
            if self.current_exhibit_index >= len(self.current_tour):
                return

            exhibit_name = self.current_tour[self.current_exhibit_index]
            description = self.exhibit_descriptions.get(exhibit_name, f"Dies ist der {exhibit_name}.")

        self.speak(f"Hier sind wir beim {exhibit_name}.")
        time.sleep(1)
        self.speak(description)
        time.sleep(1)
        self.speak("Fragen Sie gerne nach.")

        with self.state_lock:
            self.qa_mode_active = True
            self.current_exhibit_index += 1
            self.current_state = RobotState.WAITING_FOR_COMMAND

    def cancel_tour(self):
        """Cancel current tour"""
        with self.state_lock:
            if self.current_state == RobotState.NAVIGATING:
                try:
                    self.nav.cancelTask()
                except:
                    pass

            self.speak("Tour abgebrochen. Sagen Sie Hey Computer.")
            self.current_state = RobotState.STANDBY
            self.current_tour = []
            self.current_exhibit_index = 0
            self.qa_mode_active = False
            self.conversation_history = []

    def reset_idle_timer(self):
        """Reset the idle timer"""
        self.cancel_idle_timer()

        with self.idle_timer_lock:
            self.idle_timer = Timer(self.idle_timeout, self.on_idle_timeout)
            self.idle_timer.start()

    def cancel_idle_timer(self):
        """Cancel idle timer"""
        with self.idle_timer_lock:
            if self.idle_timer is not None:
                self.idle_timer.cancel()
                self.idle_timer = None

    def on_idle_timeout(self):
        """Handle idle timeout"""
        self.get_logger().info('Idle timeout - returning home')
        self.return_to_home()

    def return_to_home(self):
        """Return to home position"""
        with self.state_lock:
            if self.current_state != RobotState.STANDBY:
                return

            self.current_state = RobotState.NAVIGATING

        goal_pose = self.create_pose(self.home_x, self.home_y, self.home_theta)

        self.get_logger().info('Returning home')
        self.nav.goToPose(goal_pose)

        while not self.nav.isTaskComplete():
            time.sleep(0.1)

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.speak("Zurück zu Hause, bereit für den nächsten Besucher.")

        with self.state_lock:
            self.current_state = RobotState.STANDBY

    def destroy_node(self):
        """Cleanup - properly shutdown without killing TTS process"""
        self.get_logger().info('Shutting down Car Museum Guide')
        
        # Cancel idle timer
        self.cancel_idle_timer()

        # Shutdown TTS worker gracefully
        if self.tts_process:
            try:
                self.get_logger().info('Stopping TTS worker')
                self.tts_process.stdin.write("SHUTDOWN\n")
                self.tts_process.stdin.flush()
                self.tts_process.terminate()
                self.tts_process.wait(timeout=3)
                self.get_logger().info('TTS worker stopped')
            except Exception as e:
                self.get_logger().warn(f'Error stopping TTS worker: {e}')
                try:
                    self.tts_process.kill()
                except:
                    pass

        # CRITICAL: Do NOT call nav.lifecycleShutdown()
        # This would kill the TTS process due to ROS2 lifecycle management
        # The Nav2 lifecycle will be managed by the system
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    museum_guide = None

    try:
        museum_guide = CarMuseumGuideBot()
        executor = MultiThreadedExecutor()
        executor.add_node(museum_guide)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if museum_guide:
            museum_guide.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()