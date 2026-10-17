from pathlib import Path
from enum import IntEnum
import json

NAMESPACEWORKERS = "/AudioWorkers"

AUDIOCREATIONNODENAME="AudioCreationWorker"
AUDIOCREATIONACTIONNAME="audioCreation"

AUDIOPLAYERNODENAME="AudioPlayerWorker"
AUDIOPLAYERACTIONNAME="audioPlayer"

AUDIOLISTENERNODENAME="AudioListenerWorker"
AUDIOLISTENERACTIONNAME="audioListener"

VOICE_PATH = '/home/ubuntu/.venv/piper-voices/de_DE-thorsten-medium.onnx' # sample rate: 22050; channels: 1
CURRENT_VOICE_CHANNELS = 1
with open(VOICE_PATH + ".json") as f:
    CURRENT_VOICE_SAMPLE_RATE = json.load(f)["audio"]["sample_rate"]
    
VOICE_CHANNELS = 2 # upsampling for jabra
VOICE_SAMPLE_RATE = 44100 # upsampling for jabra
AUDIO_DEVICE = "Jabra"

INPUT_SAMPLE_RATE = 16000
INPUT_CHANNELS = 1
INPUT_BLOCKSIZE = 1024
SILENCE_THRESHOLD = 0.01
PRE_START_RECORDING_TIME = 0.3

VOLUME = 1.0
CHUNK_S = 0.1

class TTS_PROGRESS(IntEnum):
    INITIAL = 0
    SYNTHESIZED = 1
    OUTPUT_CREATED = 2