import time
import soundfile as sf
import sounddevice as sd
import queue
import os
from pathlib import Path
import uuid
import numpy as np
from threading import Lock, Event
from collections import deque

import rclpy
from rclpy.action.server import ActionServer, ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from speaking_interfaces.action import AudioListener

import constants

class AudioListenerActionServer(Node):
    
    def __init__(self):
        super().__init__(constants.AUDIOLISTENERNODENAME, namespace=constants.NAMESPACEWORKERS)
        
        self.__sr = constants.INPUT_SAMPLE_RATE
        self.__bs = constants.INPUT_BLOCKSIZE
        self.__ch = constants.INPUT_CHANNELS
        self.audio_queue = queue.Queue()
        self.lock_listener_node = Lock()
        self.preroll_lock = Lock()
        self.recording_event = Event()
        self.preroll = deque(maxlen=int(constants.PRE_START_RECORDING_TIME * self.__sr/self.__bs))
        
        
        self.TIME_PER_BLOCK = self.__bs/self.__sr
        
        def callback(indata, frames, info_time, status):
            if status:
                self.get_logger().info(f"{status}")
            
            block = indata.copy()
            with self.preroll_lock:
                self.preroll.append(block) 
                if self.recording_event.is_set():
                    self.audio_queue.put(block)
        
        self.stream = sd.InputStream(
            samplerate=self.__sr,
            blocksize=self.__bs,
            latency="high",
            channels=self.__ch,
            device=constants.AUDIO_DEVICE,
            callback=callback
        )
        
        self.stream.start()
        
        ros_home = Path(os.environ.get('ROS_HOME', Path.home() / '.ros'))
        self.__audio_dir = ros_home / 'speaking_service' / 'recorded_audio_cache'
        
        self.get_logger().info("Starting Server ...")
        self.__action_server = ActionServer(
            node=self, 
            action_type=AudioListener, 
            action_name=constants.AUDIOLISTENERACTIONNAME,
            execute_callback=self.execute_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            goal_callback=self.goal_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_logger().info("Server started!")
        
    def execute_callback(self, goal_handle: ServerGoalHandle) -> AudioListener.Result:
        file_path = self.__audio_dir / f'{uuid.uuid4().hex}.wav'
        keep_file = False
        
        try:
            start_time = time.perf_counter()
            self.get_logger().info("Executing goal ...")
            request: AudioListener.Goal = goal_handle.request
            
            if not self.stream.active:
                self.get_logger().error("No valid input stream: Please restart!")
                goal_handle.abort()
                return self.assemble_result(False, "NoAudioStream", "", 0.0) 
            
            start_timeout_blocks = int(request.start_timeout * self.__sr/self.__bs)
            max_duration_blocks = int(request.max_duration * self.__sr/self.__bs)
            end_timeout_blocks = int(request.silence_duration * self.__sr/self.__bs)
            
            total_blocks = 0
            start_silence_blocks = 0
            end_silence_blocks = 0
            heard_voice = False
            
            get_total_recorded_time = lambda : (total_blocks - start_silence_blocks) * self.TIME_PER_BLOCK
            
            feedback = AudioListener.Feedback()

            with sf.SoundFile(file_path, mode="wb", samplerate=self.__sr, channels=self.__ch) as f:
                self.get_logger().info("Start recording...")
                self.get_logger().info(f"Started recording after: {((time.perf_counter() - start_time)*1000):.2f} ms")   
                 
                # load preroll data
                with self.preroll_lock:
                    for block in self.preroll:
                        self.audio_queue.put(block)
                    self.recording_event.set()
                
                while end_silence_blocks < end_timeout_blocks:
                    # check for cancelation
                    if goal_handle.is_cancel_requested:
                        self.get_logger().info("Cancel request recieved")
                        goal_handle.canceled()
                        keep_file = heard_voice
                        if heard_voice:
                            return self.assemble_result(False, "Canceled", str(file_path), get_total_recorded_time())
                        else:
                            return self.assemble_result(False, "Canceled", "", 0.0)
                    
                    # get current data
                    try:
                        block = self.audio_queue.get(timeout=5.0)
                    except queue.Empty:
                        self.get_logger().error("Main Queue Timeout")
                        goal_handle.abort()
                        return self.assemble_result(False, "MainQueueTimeout", "", 0.0)
                    
                    # Check if voice heard
                    block_silent = self.block_silent(block)
                    if not heard_voice and not block_silent:
                        heard_voice = True
                    
                    if heard_voice:
                        if block_silent:
                            end_silence_blocks += 1
                        else:
                            end_silence_blocks = 0
                        
                        feedback.state, feedback.recorded_time = AudioListener.Feedback.RECORDING, get_total_recorded_time()
                        goal_handle.publish_feedback(feedback)
                        
                        f.write(block)
                    else:
                        start_silence_blocks += 1
                        
                        feedback.state, feedback.recorded_time = AudioListener.Feedback.WAITING_FOR_SPEECH, 0.0
                        goal_handle.publish_feedback(feedback)
                        
                        if start_silence_blocks >= start_timeout_blocks:
                            self.get_logger().info("Time exceeds starting timeout. Aborting")
                            goal_handle.abort()
                            return self.assemble_result(False, "StartTimeout", "",0.0)
                    
                    # evaluate total blocks
                    total_blocks += 1
                    if total_blocks >= max_duration_blocks:
                        self.get_logger().info("Time exceeds total timeout. Aborting")
                        goal_handle.abort()
                        keep_file = heard_voice
                        if heard_voice:
                            return self.assemble_result(False, "TotalTimeout", str(file_path),get_total_recorded_time())
                        else:
                            return self.assemble_result(False, "TotalTimeout", "",0.0)
                        
                self.get_logger().info("Stop recording...")
                
            goal_handle.succeed()
            keep_file = True
            return self.assemble_result(True, "FinishedRecording", str(file_path), get_total_recorded_time())

        except Exception as e:
            self.get_logger().error(f"{e}")
            goal_handle.abort()
            return self.assemble_result(False, f"Error: {e}", "", 0.0)
            
        finally:
            with self.preroll_lock:
                self.recording_event.clear()
            self._drain_queue()
            
            if not keep_file:
                file_path.unlink(missing_ok=True)
                
            self.lock_listener_node.release()
        
    def block_silent(self, block):
        rms = np.sqrt(np.mean(block ** 2))
        return rms < constants.SILENCE_THRESHOLD
    
    def goal_callback(self, goal_request):
        if not self.lock_listener_node.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT
    
    def assemble_result(self, success: bool, message: str, file_name: str, duration: float):
        result = AudioListener.Result()
        result.success = success
        result.message = message
        result.file_name = file_name
        result.duration = duration
        return result
    
    def _drain_queue(self):
        try:
            while True:
                self.audio_queue.get_nowait()
        except queue.Empty:
            pass
    
    def destroy_node(self):
        self.stream.stop()
        self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    audio_creation_action_server = AudioListenerActionServer()
    
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
