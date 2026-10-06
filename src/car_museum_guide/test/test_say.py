import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import wave

import pytest

SAY_PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'say.py'
spec = importlib.util.spec_from_file_location('say', SAY_PATH)
say = importlib.util.module_from_spec(spec)
spec.loader.exec_module(say)


def add_card(asound_root, index, card_id, usb=False, playback=True):
    card = asound_root / f'card{index}'
    card.mkdir(parents=True)
    (card / 'id').write_text(card_id + '\n')
    if usb:
        (card / 'usbid').write_text('047f:c058\n')
    if playback:
        (card / 'pcm0p').mkdir()


class FakeAplay:
    """Stands in for aplay, the system boundary; keeps what it was asked to play."""

    def __init__(self):
        self.played = []

    def __call__(self, wav_path, device):
        with wave.open(str(wav_path), 'rb') as wav:
            frames = wav.readframes(wav.getnframes())
            self.seconds = wav.getnframes() / wav.getframerate()
        self.played.append((device, frames))


def test_usb_speaker_is_found_next_to_the_headphone_jack(tmp_path):
    add_card(tmp_path, 0, 'Headphones')
    add_card(tmp_path, 1, 'Seri', usb=True)

    assert say.find_usb_speaker(tmp_path) == 'plughw:CARD=Seri,DEV=0'


def test_usb_microphone_without_playback_is_not_a_speaker(tmp_path):
    add_card(tmp_path, 1, 'Webcam', usb=True, playback=False)
    add_card(tmp_path, 2, 'Seri', usb=True)

    assert say.find_usb_speaker(tmp_path) == 'plughw:CARD=Seri,DEV=0'


def test_without_usb_speaker_nothing_is_played_and_the_error_says_so(tmp_path, capsys):
    add_card(tmp_path, 0, 'Headphones')
    aplay = FakeAplay()

    exit_code = say.main(['Hallo'], asound_root=tmp_path, player=aplay)

    assert exit_code == 1
    assert aplay.played == []
    assert 'USB' in capsys.readouterr().err


needs_default_voice = pytest.mark.skipif(
    importlib.util.find_spec('piper') is None
    or not (say.VOICES_DIR / f'{say.DEFAULT_VOICE}.onnx').exists(),
    reason='needs piper-tts and the default voice (run in /home/ubuntu/.venv/turtlebot_venv)')


def peak(frames):
    samples = memoryview(frames).cast('h')
    return max(abs(s) for s in samples)


@needs_default_voice
def test_text_is_spoken_on_the_usb_speaker(tmp_path):
    add_card(tmp_path, 0, 'Headphones')
    add_card(tmp_path, 1, 'Seri', usb=True)
    aplay = FakeAplay()

    exit_code = say.main(['Hallo, ich bin Ihr Museumsführer.'],
                         asound_root=tmp_path, player=aplay)

    assert exit_code == 0
    assert len(aplay.played) == 1
    device, frames = aplay.played[0]
    assert device == 'plughw:CARD=Seri,DEV=0'
    assert peak(frames) > 1000
    assert aplay.seconds > 1.5  # the whole sentence, not just a word


@needs_default_voice
def test_lower_volume_is_played_quieter(tmp_path):
    add_card(tmp_path, 1, 'Seri', usb=True)
    aplay = FakeAplay()

    say.main(['Guten Tag.'], asound_root=tmp_path, player=aplay)
    say.main(['Guten Tag.', '--volume', '0.3'], asound_root=tmp_path, player=aplay)

    normal, quiet = (peak(frames) for _, frames in aplay.played)
    assert quiet < normal * 0.5


def test_voice_that_is_not_downloaded_is_refused(tmp_path, capsys):
    add_card(tmp_path, 1, 'Seri', usb=True)
    aplay = FakeAplay()

    exit_code = say.main(['Hallo', '--voice', 'de_DE-nobody-low'],
                         asound_root=tmp_path, player=aplay)

    assert exit_code == 1
    assert aplay.played == []
    assert 'de_DE-nobody-low' in capsys.readouterr().err


@pytest.mark.parametrize('volume', ['inf', 'nan', '0', '-1', '1.5'])
def test_volume_outside_what_the_speaker_can_play_is_refused(tmp_path, volume):
    add_card(tmp_path, 1, 'Seri', usb=True)
    aplay = FakeAplay()

    with pytest.raises(SystemExit) as refused:
        say.main(['Hallo', '--volume', volume], asound_root=tmp_path, player=aplay)

    assert refused.value.code == 2
    assert aplay.played == []


@needs_default_voice
def test_text_with_nothing_to_say_is_refused(tmp_path, capsys):
    add_card(tmp_path, 1, 'Seri', usb=True)
    aplay = FakeAplay()

    exit_code = say.main(['...'], asound_root=tmp_path, player=aplay)

    assert exit_code == 1
    assert aplay.played == []
    assert 'nothing to say' in capsys.readouterr().err


def busy_aplay(wav_path, device):
    """Behaves like aplay when another sound is still playing on the device."""
    raise subprocess.CalledProcessError(1, ['aplay', '-D', device, str(wav_path)])


@needs_default_voice
def test_speaker_that_cannot_play_is_reported(tmp_path, capsys):
    add_card(tmp_path, 1, 'Seri', usb=True)

    exit_code = say.main(['Hallo'], asound_root=tmp_path, player=busy_aplay)

    assert exit_code == 1
    assert 'plughw:CARD=Seri,DEV=0' in capsys.readouterr().err


def interrupted_aplay(wav_path, device):
    raise KeyboardInterrupt


@needs_default_voice
def test_ctrl_c_while_speaking_stops_like_any_interrupted_command(tmp_path):
    add_card(tmp_path, 1, 'Seri', usb=True)

    exit_code = say.main(['Hallo'], asound_root=tmp_path, player=interrupted_aplay)

    assert exit_code == 130


SLOW_APLAY_RUN = """
import importlib.util, pathlib, sys, time
spec = importlib.util.spec_from_file_location('say', sys.argv[1])
say = importlib.util.module_from_spec(spec)
spec.loader.exec_module(say)

def slow_aplay(wav_path, device):
    pathlib.Path(sys.argv[3]).touch()
    time.sleep(60)

sys.exit(say.main(['Hallo'], asound_root=pathlib.Path(sys.argv[2]), player=slow_aplay))
"""


@needs_default_voice
def test_stopping_with_sigterm_leaves_no_wav_behind(tmp_path):
    asound_root, tmp, playing = tmp_path / 'asound', tmp_path / 'tmp', tmp_path / 'playing'
    add_card(asound_root, 1, 'Seri', usb=True)
    tmp.mkdir()
    run = subprocess.Popen(
        [sys.executable, '-c', SLOW_APLAY_RUN, str(SAY_PATH), str(asound_root), str(playing)],
        env={**os.environ, 'TMPDIR': str(tmp)})
    deadline = time.monotonic() + 60
    while not playing.exists() and time.monotonic() < deadline:
        time.sleep(0.1)

    run.send_signal(signal.SIGTERM)
    run.wait(timeout=10)

    assert list(tmp.rglob('*.wav')) == []
