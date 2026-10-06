#!/usr/bin/env python3
"""Speak German text on the robot's USB speaker with Piper."""

import argparse
from pathlib import Path
import signal
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


def volume(text):
    """Parse --volume, refusing values the speaker cannot play."""
    value = float(text)
    if not 0 < value <= 1:  # Piper already normalizes to full scale at 1
        raise argparse.ArgumentTypeError(f'{text} is not between 0 and 1')
    return value


def aplay(wav_path, device):
    """Play a WAV file on an ALSA device and wait until it has finished."""
    subprocess.run(['aplay', '-q', '-D', device, str(wav_path)], check=True)


def synthesize(text, wav_path, voice_path, volume):
    """Write the spoken text to wav_path; return False when there is nothing to say."""
    from piper import PiperVoice, SynthesisConfig

    voice = PiperVoice.load(str(voice_path))
    chunks = list(voice.synthesize(text, syn_config=SynthesisConfig(volume=volume)))
    if not chunks:
        return False
    with wave.open(str(wav_path), 'wb') as wav_file:
        wav_file.setframerate(chunks[0].sample_rate)
        wav_file.setsampwidth(chunks[0].sample_width)
        wav_file.setnchannels(chunks[0].sample_channels)
        for chunk in chunks:
            wav_file.writeframes(chunk.audio_int16_bytes)
    return True


def speak(argv, asound_root, player):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('text', help='what the robot should say')
    parser.add_argument('--voice', default=DEFAULT_VOICE,
                        help=f'Piper voice in {VOICES_DIR} (default: %(default)s)')
    parser.add_argument('--volume', type=volume, default=1.0,
                        help='between 0 and 1 (default: 1, full scale); e.g. 0.5 is quieter')
    args = parser.parse_args(argv)

    device = find_usb_speaker(asound_root)
    if device is None:
        print('say.py: no USB speaker found; plug it in and try again.', file=sys.stderr)
        return 1

    voice_path = VOICES_DIR / f'{args.voice}.onnx'
    if not voice_path.exists():
        print(f'say.py: voice {args.voice} not found in {VOICES_DIR}; download it with\n'
              f'  python -m piper.download_voices --data-dir {VOICES_DIR} {args.voice}',
              file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory() as tmp:
        wav_path = Path(tmp) / 'say.wav'
        if not synthesize(args.text, wav_path, voice_path, args.volume):
            print(f'say.py: nothing to say in {args.text!r}', file=sys.stderr)
            return 1
        try:
            player(wav_path, device)
        except subprocess.CalledProcessError as error:
            print(f'say.py: aplay could not play on {device} (exit {error.returncode})',
                  file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            return 130  # the shell's convention for a command stopped by Ctrl-C
    return 0


def exit_on_signal(signum, frame):
    sys.exit(128 + signum)


def main(argv=None, asound_root=Path('/proc/asound'), player=aplay):
    # SIGTERM (e.g. from ros2 launch) must unwind normally so the temporary WAV is removed.
    previous = signal.signal(signal.SIGTERM, exit_on_signal)
    try:
        return speak(argv, asound_root, player)
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == '__main__':
    sys.exit(main())
