#!/usr/bin/env python3
"""Simple offline speech smoke test: record audio, transcribe with faster-whisper, and TTS a phrase with Coqui."""
import sounddevice as sd
import soundfile as sf
import tempfile
from faster_whisper import WhisperModel
from TTS.api import TTS

SR = 16000
DUR = 3.0

print('Recording for', DUR, 'seconds...')
audio = sd.rec(int(DUR * SR), samplerate=SR, channels=1, dtype='int16')
sd.wait()

with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
    path = f.name
    sf.write(path, audio, SR, subtype='PCM_16')

print('Loading whisper tiny...')
model = WhisperModel('tiny', device='cpu', compute_type='int8')
segments, info = model.transcribe(path, beam_size=5)
text = ''.join([seg.text for seg in segments]).strip()
print('Transcription:', text)

print('Running TTS...')
tts = TTS('tts_models/en/ljspeech/tacotron2-DDC', progress_bar=False, gpu=False)
with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
    tpath = f.name
    tts.tts_to_file(text='Hello, this is a smoke test from your TurtleBot guide.', file_path=tpath)

import simpleaudio as sa
wave_obj = sa.WaveObject.from_wave_file(tpath)
play = wave_obj.play()
play.wait_done()
print('Done')