#!/usr/bin/env python3
"""
Speech Recognition Node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import subprocess
import threading
import sys
import os
from typing import Optional
from pathlib import Path

# Constants
DEFAULT_AUTO_RESTART = True
HEALTH_CHECK_INTERVAL = 5.0
READY_SIGNAL = "READY"
WAKE_EVENT = "EVENT:WAKE"
COMMAND_EVENT_PREFIX = "EVENT:COMMAND:"
LOG_PREFIX = "LOG:"
WARN_PREFIX = "WARN:"
ERROR_PREFIX = "ERROR:"
FATAL_PREFIX = "FATAL:"
SHUTDOWN_TIMEOUT = 3


class SpeechRecognitionNodeError(Exception):
    """Custom exception for speech recognition node errors"""
    pass


class WorkerProcessManager:
    """Manages the speech worker subprocess"""
    
    def __init__(self, python_path: str, script_path: str, logger):
        """
        Initialize worker process manager.
        
        Args:
            python_path: Path to Python executable
            script_path: Path to worker script
            logger: ROS2 logger instance
        """
        self._python_path = python_path
        self._script_path = script_path
        self._logger = logger
        self._process: Optional[subprocess.Popen] = None
        self._should_run = True
        self._stdout_thread: Optional[threading.Thread] = None
        self._stderr_thread: Optional[threading.Thread] = None
        
        self._validate_paths()
    
    def _validate_paths(self) -> None:
        """Validate that required paths exist"""
        if not os.path.exists(self._python_path):
            raise SpeechRecognitionNodeError(
                f'Python executable not found: {self._python_path}'
            )
        
        if not os.path.exists(self._script_path):
            raise SpeechRecognitionNodeError(
                f'Worker script not found: {self._script_path}'
            )
    
    def start(self, stdout_callback, stderr_callback) -> None:
        """
        Start the worker process.
        
        Args:
            stdout_callback: Callback for stdout lines
            stderr_callback: Callback for stderr lines
        """
        self._logger.info('Starting speech worker...')
        self._logger.info(f'Python: {self._python_path}')
        self._logger.info(f'Script: {self._script_path}')
        
        try:
            self._process = subprocess.Popen(
                [self._python_path, self._script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=subprocess.PIPE,
                text=True,
                bufsize=1
            )
        except Exception as e:
            raise SpeechRecognitionNodeError(f'Failed to start worker: {e}')
        
        self._should_run = True
        
        self._stdout_thread = threading.Thread(
            target=self._monitor_stream,
            args=(self._process.stdout, stdout_callback),
            daemon=True
        )
        self._stdout_thread.start()
        
        self._stderr_thread = threading.Thread(
            target=self._monitor_stream,
            args=(self._process.stderr, stderr_callback),
            daemon=True
        )
        self._stderr_thread.start()
        
        self._logger.info('Speech worker process started, waiting for READY signal...')
    
    def _monitor_stream(self, stream, callback) -> None:
        """
        Monitor a stream and call callback for each line.
        
        Args:
            stream: Stream to monitor
            callback: Callback function for each line
        """
        while self._should_run and self._process and self._process.poll() is None:
            try:
                line = stream.readline()
                
                if not line:
                    break
                
                line = line.strip()
                
                if line:
                    callback(line)
            
            except Exception as e:
                if self._should_run:
                    self._logger.error(f'Error reading stream: {e}')
                break
    
    def is_alive(self) -> bool:
        """
        Check if worker process is alive.
        
        Returns:
            True if process is running, False otherwise
        """
        return self._process is not None and self._process.poll() is None
    
    def stop(self) -> None:
        """Stop the worker process gracefully"""
        self._logger.info('Shutting down speech worker...')
        
        self._should_run = False
        
        if self._process:
            try:
                self._process.terminate()
                
                try:
                    self._process.wait(timeout=SHUTDOWN_TIMEOUT)
                    self._logger.info('Speech worker terminated gracefully')
                except subprocess.TimeoutExpired:
                    self._logger.warning('Speech worker did not terminate, killing...')
                    self._process.kill()
                    self._process.wait()
            except Exception as e:
                self._logger.error(f'Error terminating worker: {e}')


class SpeechRecognitionNode(Node):
    """ROS2 node for speech recognition"""
    
    def __init__(self):
        """Initialize the speech recognition node"""
        super().__init__('speech_recognition_node')
        
        self._declare_parameters()
        python_path, script_path, auto_restart = self._load_parameters()
        
        self._auto_restart = auto_restart
        
        self._setup_publishers()
        
        self._worker_manager = WorkerProcessManager(
            python_path,
            script_path,
            self.get_logger()
        )
        
        self._worker_manager.start(
            self._handle_stdout,
            self._handle_stderr
        )
        
        self._health_timer = self.create_timer(
            HEALTH_CHECK_INTERVAL,
            self._check_worker_health
        )
        
        self.get_logger().info(
            'Speech Recognition Node initialized '
        )
    
    def _declare_parameters(self) -> None:
        """Declare ROS2 parameters"""
        self.declare_parameter('venv_python_path', '')
        self.declare_parameter('worker_script_path', '')
        self.declare_parameter('auto_restart', DEFAULT_AUTO_RESTART)
    
    def _load_parameters(self) -> tuple[str, str, bool]:
        """
        Load and validate parameters.
        
        Returns:
            Tuple of (python_path, script_path, auto_restart)
            
        Raises:
            ValueError: If required parameters are missing
        """
        python_path = self.get_parameter('venv_python_path').value
        script_path = self.get_parameter('worker_script_path').value
        auto_restart = self.get_parameter('auto_restart').value
        
        if not python_path:
            raise ValueError('Required parameter "venv_python_path" not set')
        
        if not script_path:
            raise ValueError('Required parameter "worker_script_path" not set')
        
        return python_path, script_path, auto_restart
    
    def _setup_publishers(self) -> None:
        """Setup ROS2 publishers"""
        self._command_pub = self.create_publisher(String, 'voice_command', 10)
        self._wake_word_pub = self.create_publisher(Bool, 'wake_word_detected', 10)
    
    def _handle_stdout(self, line: str) -> None:
        """
        Handle stdout line from worker.
        
        Args:
            line: Output line from worker
        """
        if line == READY_SIGNAL:
            self.get_logger().info('Speech worker is READY')
        
        elif line == WAKE_EVENT:
            self.get_logger().info('Wake word detected')
            msg = Bool()
            msg.data = True
            self._wake_word_pub.publish(msg)
        
        elif line.startswith(COMMAND_EVENT_PREFIX):
            text = line.replace(COMMAND_EVENT_PREFIX, "").strip()
            
            if text:
                self.get_logger().info(f'Voice command: "{text}"')
                msg = String()
                msg.data = text
                self._command_pub.publish(msg)
        
        else:
            self.get_logger().debug(f'Worker output: {line}')
    
    def _handle_stderr(self, line: str) -> None:
        """
        Handle stderr line from worker.
        
        Args:
            line: Error line from worker
        """
        if line.startswith(LOG_PREFIX):
            self.get_logger().info(f'[Worker] {line[len(LOG_PREFIX):].strip()}')
        elif line.startswith(WARN_PREFIX):
            self.get_logger().warning(f'[Worker] {line[len(WARN_PREFIX):].strip()}')
        elif line.startswith(ERROR_PREFIX):
            self.get_logger().error(f'[Worker] {line[len(ERROR_PREFIX):].strip()}')
        elif line.startswith(FATAL_PREFIX):
            self.get_logger().fatal(f'[Worker] {line[len(FATAL_PREFIX):].strip()}')
        else:
            self.get_logger().info(f'[Worker] {line}')
    
    def _check_worker_health(self) -> None:
        """Periodic health check of worker process"""
        if not self._worker_manager.is_alive():
            self.get_logger().error('Speech worker process is dead')
            
            if self._auto_restart:
                self.get_logger().info('Attempting to restart speech worker...')
                try:
                    self._worker_manager.start(
                        self._handle_stdout,
                        self._handle_stderr
                    )
                except SpeechRecognitionNodeError as e:
                    self.get_logger().error(f'Failed to restart worker: {e}')
    
    def destroy_node(self) -> None:
        """Clean shutdown of node"""
        self._worker_manager.stop()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechRecognitionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in speech recognition node: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
