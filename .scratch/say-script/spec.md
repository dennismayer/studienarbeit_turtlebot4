# Spec: make the robot talk (say.py)

Status: in progress (implementation done; QA round 3 failed on N1b; code review not yet run)
Branch: feat/say-script (base: master @ 8205ec6, the baseline commit)

Settled in /grill-with-docs on 2026-10-06.

## Requirements

1. A plain CLI script at `src/car_museum_guide/scripts/say.py`. No ROS.
2. `say.py "Hallo, ich bin Ihr Museumsführer."` speaks the text and exits once playback has finished (blocking).
3. TTS engine: Piper (offline neural, `piper-tts` Python package). No internet needed at runtime.
4. Default voice `de_DE-thorsten-medium`; a `--voice` flag selects another Piper voice. German only.
5. Playback on the USB speaker via `aplay` (WAV). The device is found by its USB property, not a hard-coded card number like the old `hw:1,0`.
6. If no USB sound card is present, exit with a clear error. Never silently fall back to the Pi headphone jack.
7. The script does not change the system mixer volume; it offers a `--volume` flag instead.
8. Python environment: the existing venv `/home/ubuntu/.venv/turtlebot_venv`.
9. Voice files stored next to the venv (`/home/ubuntu/.venv/piper-voices`), outside the repo.
10. Out of scope: wiring into the guide node / ROS.

## Progress

### Done
- Installed `piper-tts==1.8.0` (vetted: PyPI attestation from OHF-Voice/piper1-gpl publish.yml) and `pytest` into turtlebot_venv. Downloaded the `de_DE-thorsten-medium` voice.
- `say.py` and `test/test_say.py` implemented test-first; mutation-checked.
- QA round 1 (at eb63552): fails F1 (`--volume`), F2 (empty text), F3 (aplay failure / Ctrl-C) — all fixed.
- QA round 2 (at 0e4f43e): F1–F3 confirmed fixed; new fail N1 (Ctrl-C during model load/synthesis) — fixed in a4a249d.
- QA round 3 (at a4a249d): N1 fixed everywhere except N1b — a Ctrl-C/SIGTERM landing just after synthesis is swallowed inside a `__del__`, so the text is spoken anyway and the exit code is 0 (pre-existing). See `qa-report-3.md`.
- Reports: `qa-report-1.md`, `qa-report-2-summary.md`, `qa-report-3.md` in this folder.

### Next steps
1. Fix N1b test-first (see `qa-report-3.md`, "Likely fix"): after synthesize() and after speak(), honour a stop signal recorded in `stopped_by` even when no exception surfaced. Then re-run QA (round 4), scoped to the fix diff.
2. Run `/code-review` on the branch since 8205ec6 with this file as the spec, then fix its findings.
3. Have a human listen: nobody has heard the audio yet (QA verified playback only via /proc/asound and WAV levels).
4. Ship (merge `feat/say-script` into `master`).

### How to run
- Speak: `src/car_museum_guide/scripts/say.py "Hallo"` (shebang points at turtlebot_venv).
- Tests: `cd src/car_museum_guide && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/ubuntu/.venv/turtlebot_venv/bin/python -m pytest -q test/test_say.py`
  (ROS's pytest plugins fail to import inside the venv, so turn plugin autoload off.)
- Lint: `ament_flake8 scripts/say.py test/test_say.py`. Other files in the package already had 719 lint errors before this change.

### Open concerns for the guide-node round (not part of this spec)
- About 8–9 s before any sound: the 63 MB model is loaded per call and the whole text is synthesized before playback. The guide will want a long-running worker.
- A second call made while the first is still speaking fails with "device busy" instead of waiting.
- `--voice` accepts path-like names and non-German voices.
- Tiny volumes (e.g. 1e-9) are accepted and play silence.
- `requirements.txt` (the old lock file) pins numpy 1.26.4, but turtlebot_venv has 2.5.3.
- `params.yaml` still points at non-existent paths (`/home/ubuntu/venv_museum`, `/home/ubuntu/car_museum_guide_ws`).
