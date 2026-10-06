# QA report 1 — say.py (tested at eb63552, fixed point 8205ec6)
Verdict: Fails QA. All 10 spec requirements pass (R6 only via main(asound_root=...); a true CLI unplug was blocked).
Evidence dir: /tmp/tmp.qzwxRty4mF/ (scenarios.md, ev/, harness scripts run_mon.sh, s7.py, s11.sh, wavstat.py).
Audio observed via polling /proc/asound/card{0,1}/pcm0p/sub0/status every 0.1s plus amixer -c0/-c1 snapshots; a pass-through aplay wrapper on PATH copied WAVs.

## Fails
F1 --volume accepts nonsensical values silently. Steps: say.py "Guten Tag, willkommen im Museum." --volume X.
  inf -> rc0, full-scale square wave played; nan -> rc0 silent + numpy RuntimeWarning; 0 -> silent; -1 -> normal loudness; 2.0/5 -> clipping.
F2 Empty/unspeakable text crashes with traceback. Steps: say.py "" / "   " / "..." -> rc1, wave.Error: # channels not specified.
F3 aplay failures as raw tracebacks.
  (a) SIGINT to process group during playback -> aplay "Interrupted system call" + CalledProcessError traceback, rc1 (expected 130-ish, no traceback).
  (b) two concurrent calls -> second: "aplay: audio open error: Device or resource busy" + traceback, rc1, its sentence never spoken.

## Concerns
C1 ./say.py with #!/usr/bin/env python3 fails outside venv (ModuleNotFoundError piper).
C2 Latency ~7-8 s before a short sentence; ~20 s before a 4-sentence paragraph (model loaded per call, whole text synthesized before playback).
C3 --voice accepts ../x and absolute paths; doesn't enforce German.
C4 SIGTERM leaves /tmp/tmpXXXX/say.wav behind.
C5 Download hint says `python -m ...`; no `python` on PATH outside venv.
C6 test/test_say.py fails 2/6 under system python3 (no piper); requirements.txt lacks piper-tts.
C7 help "0.5 for half as loud" is amplitude, not perceived loudness.

## Blocked
R6 at the CLI level with the headset truly unplugged (needs unplug or root bind-mount over /proc/asound).
