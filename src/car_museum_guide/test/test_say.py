import importlib.util
from pathlib import Path
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
    not (say.VOICES_DIR / f'{say.DEFAULT_VOICE}.onnx').exists(),
    reason='default Piper voice not downloaded')


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
