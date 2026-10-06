#!/usr/bin/env python3
"""Speak German text on the robot's USB speaker with Piper."""

import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
import wave

VOICES_DIR = Path('/home/ubuntu/.venv/piper-voices')
DEFAULT_VOICE = 'de_DE-thorsten-medium'


def find_usb_speaker(asound_root=Path('/proc/asound')):
    """Return the ALSA device of the first USB sound card."""
    for card in sorted(asound_root.glob('card[0-9]*')):
        if (card / 'usbid').exists() and (card / 'pcm0p').exists():
            card_id = (card / 'id').read_text().strip()
            return f'plughw:CARD={card_id},DEV=0'


def aplay(wav_path, device):
    """Play a WAV file on an ALSA device and wait until it has finished."""
    subprocess.run(['aplay', '-q', '-D', device, str(wav_path)], check=True)


def synthesize(text, wav_path):
    from piper import PiperVoice

    voice = PiperVoice.load(str(VOICES_DIR / f'{DEFAULT_VOICE}.onnx'))
    with wave.open(str(wav_path), 'wb') as wav_file:
        voice.synthesize_wav(text, wav_file)


def main(argv=None, asound_root=Path('/proc/asound'), player=aplay):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('text', help='what the robot should say')
    args = parser.parse_args(argv)

    device = find_usb_speaker(asound_root)
    if device is None:
        print('say.py: no USB speaker found; plug it in and try again.', file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory() as tmp:
        wav_path = Path(tmp) / 'say.wav'
        synthesize(args.text, wav_path)
        player(wav_path, device)
    return 0


if __name__ == '__main__':
    sys.exit(main())
