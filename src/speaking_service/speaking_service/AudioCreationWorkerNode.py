import time
from piper import PiperVoice, SynthesisConfig
import os
import uuid
from pathlib import Path
import soundfile as sf
import numpy as np

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from speaking_interfaces.action import AudioCreation

import constants
from constants import TTS_PROGRESS

class AudioCreationActionServer(Node):
    
    def __init__(self):
        super().__init__(constants.AUDIOCREATIONNODENAME, namespace=constants.NAMESPACEWORKERS)
        
        # load voice 
        # TODO error management
        self.__voice = PiperVoice.load(constants.VOICE_PATH)
        self.get_logger().info("Voice loaded!")
        
        # TODO write in constants or even as parameter?
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self.__audio_dir = ros_home / 'speaking_service' / 'audio_cache'
        self.__sr = constants.CURRENT_VOICE_SAMPLE_RATE
        self.__ch = constants.CURRENT_VOICE_CHANNELS
                
        # Start the server
        self.get_logger().info("Starting Server ...")
        # using a Reentrant Callback Group because audio creation can be done in parallel 
        # at least theoretically -> should not appear in real life
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioCreation, 
            action_name=constants.AUDIOCREATIONACTIONNAME,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info("Server started!")
        
    def execute_callback(self, goal_handle: ServerGoalHandle) -> AudioCreation.Result:
        self.get_logger().info("Executing goal ...")
        request: AudioCreation.Goal = goal_handle.request
        self.get_logger().info(f"Text to Transform: {request.text}")
        
        # abort if text is empty
        if not request.text or request.text.strip() == "":
            self.get_logger().error("Text is empty!")
            goal_handle.abort()
            return AudioCreation.Result()
        
        # Initial feedback
        feedback_msg = AudioCreation.Feedback()
        feedback_msg.status = TTS_PROGRESS.INITIAL
        goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info("Start TTS...")
        chunks = list(self.__voice.synthesize(request.text, syn_config=SynthesisConfig(volume=constants.VOLUME)))
        
        # update feedback to synthesized
        feedback_msg.status = TTS_PROGRESS.SYNTHESIZED
        goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info("Finished with synthesizing...")
        file_path = self.__audio_dir / f'{uuid.uuid4().hex}.wav'
        
        self.get_logger().info(f"SR: {self.__sr} Channels: {self.__ch} FilePath: {file_path}")
        
        with sf.SoundFile(file_path, mode="x", samplerate=self.__sr, channels=self.__ch) as f:
            for chunk in chunks:
                f.write(chunk.audio_float_array)
        
        
        feedback_msg.status = TTS_PROGRESS.OUTPUT_CREATED
        goal_handle.publish_feedback(feedback_msg)
        
        self.get_logger().info("Finished Transcription!")
        goal_handle.succeed()
        result = AudioCreation.Result()
        result.file_name = str(file_path)
        return result

def main(args=None):
    rclpy.init(args=args)
    
    audio_creation_action_server = AudioCreationActionServer()
    
    executor = MultiThreadedExecutor()
    
    try:
        executor.add_node(audio_creation_action_server)
        executor.spin()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Exception occurred in main thread: {e}")
    
    executor.shutdown()
    audio_creation_action_server.destroy_node()
    
    try:
        rclpy.shutdown()
    except:
        pass
    
if __name__ == "__main__":
    main()
