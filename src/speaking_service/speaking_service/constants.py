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

AUDIOTRANSCRIBERNODENAME="AudioTranscriberWorker"
AUDIOTRANSCRIBERACTIONNAME="audioTranscriber"

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

TRANSCRIPTION_BEAM_SIZE = 5                                 
TRANSCRIPTION_TEMPERATURE = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)  
TRANSCRIPTION_CONDITION_ON_PREVIOUS_TEXT = False
TRANSCRIPTION_VAD_FILTER = True
TRANSCRIPTION_VAD_PARAMETERS = {"min_silence_duration_ms": 1000}  # other keys keep faster-whisper defaults
TRANSCRIPTION_NO_SPEECH_THRESHOLD = 0.6                      # segment dropped if no_speech_prob > this AND avg_logprob < -1.0

TRANSCRIPTION_MODEL_SIZE = "tiny"
TRANSCRIPTION_DEVICE = "cpu"
TRANSCRIPTION_COMPUTE_TYPE = "int8"

TRANSCRIPTION_SR = 16000
TRANSCRIPTION_CH = 1


class TTS_PROGRESS(IntEnum):
    INITIAL = 0
    SYNTHESIZED = 1
    OUTPUT_CREATED = 2