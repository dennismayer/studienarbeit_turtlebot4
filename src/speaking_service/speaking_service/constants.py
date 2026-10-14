from pathlib import Path

NAMESPACEWORKERS = "/AudioWorkers"

AUDIOCREATIONNODENAME="AudioCreationWorker"
AUDIOCREATIONACTIONNAME="audioCreation"

VOICE_PATH = '/home/ubuntu/.venv/piper-voices/de_DE-thorsten-medium.onnx'
VOLUME = 1.0
LEADING_SILENCE_S = 0.3

class TTS_PROGRESS:
    INITIAL = 0
    SYNTHESIZED = 1
    OUTPUT_CREATED = 2