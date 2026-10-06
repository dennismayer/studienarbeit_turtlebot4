# QA report 2 (follow-up) — tested at 0e4f43e
Verdict: Fails QA. F1, F2, F3a, F3b, C1, C4, C5, C6 resolved; no regressions.
Evidence dir: /tmp/tmp.Maurv8lv0a/ (scenarios.md, ev/, harness: run_mon.sh, sig.sh, intpid.py, ref.py, wavstat.py).
N1 (fail): Ctrl-C / SIGINT during model load or synthesis printed a KeyboardInterrupt traceback, process died by signal.
  Steps: run say.py "Willkommen im Automuseum. Hier sehen Sie die Geschichte des Automobils." in an interactive shell (tmux), Ctrl-C after 1 s or 4 s, before audio.
  Expected: exit 130, no traceback (same as during playback).
  Implementer note: a related variant was found while fixing — when SIGINT lands during `import piper` (~0.4–0.9 s into main), onnxruntime's C extension raises ImportError('initialization failed') instead of KeyboardInterrupt.
Open concerns (not fails): C2 latency ~8-9 s/call; C3 --voice accepts paths; C7 help wording; C8 tiny volumes like 1e-9 play silence; C9 argparse `--volume -inf`; C10 concurrent call drops its sentence; C11 aplay's own "Interrupted system call" line; C12 onnxruntime litter (.ses, mat-debug-*.log) in TMPDIR.
Blocked: R6 at the CLI with the headset truly unplugged.
