#!/usr/bin/env python3
import sounddevice as sd
import soundfile as sf
import numpy as np
import queue
from faster_whisper import WhisperModel 
import time

filename = "output.wav"
sd.default.device = "Jabra"


def record_audio():
    """
    Records audio from the microphone and saves it to a file.
    """
    # Set the parameters for recording
    duration = 5  # seconds
    sample_rate = 16000  # Hz
    channels = 1  # mono

    print("Recording...")
    audio_data = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=channels)
    sd.wait()  # Wait until recording is finished
    print("Recording finished.")

    # Save the recorded audio to a file
    sf.write(filename, audio_data, sample_rate)
    print(f"Audio saved to {filename}.")
    
def play_audio():
    data, fs = sf.read(filename, dtype="float32")
    sd.play(data, fs)
    status = sd.wait()
    print(status)

SR = 16000
BS = 1024
SILENCE_BLOCK_NEEDED = int(2.0 * SR/BS)
SILENCE_THRESHOLD = 0.01

def block_silent(block):
    rms = np.sqrt(np.mean(block ** 2))
    return rms < SILENCE_THRESHOLD

def record_until_no_more_noise():
    q = queue.Queue()

    def callback(indata, frames, info_time, status):
        if status:
            print(status)
        q.put(indata.copy())
    
    with sf.SoundFile("output.wav", mode="w", samplerate=SR, channels=1) as f:
        with sd.InputStream(samplerate=SR, blocksize=BS, latency="high", channels=1, callback=callback): 
            silent_blocks, heard_voice = 0, False
            print("Start Recording")
            while silent_blocks < SILENCE_BLOCK_NEEDED:
                block = q.get()
                if block_silent(block):
                    silent_blocks += heard_voice
                else: 
                    silent_blocks, heard_voice = 0, True
                
                if heard_voice:
                    f.write(block)
            print("Stop Recording...")

def transcribe():
    model = WhisperModel("tiny", device="cpu", compute_type="int8")
    print("Start Transcription")
    start_time = time.perf_counter()
    audio, sr = sf.read(filename, dtype="float32")
    segments, info = model.transcribe(audio,"de",condition_on_previous_text=False,initial_prompt="Es wird nach einem Namen gefragt, welcher in folgender Liste ist: Bernd Lichte, Dennis Mayer, Gerd Müller")
    text = " ".join(s.text.strip() for s in segments)
    stop_time = time.perf_counter()
    total_time = stop_time - start_time
    print(f"Time for transcribing: {total_time:.2f}s")
    
    print("Der Text lautet: ", text)
    
    return text
    

# print(sd.query_devices())

# record_audio()
# play_audio()
import say


record_until_no_more_noise()
result = transcribe()
say.main(argv={result})

