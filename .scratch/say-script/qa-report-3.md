# QA report 3 (follow-up), tested at a4a249d (fix diff 0e4f43e..a4a249d)

Verdict: Fails QA.
Evidence: /tmp/tmp.C4PudwncLu/ (scenarios.md, ev/, harness scripts sweep2.py, wavtrig.py, wavtrig2.py, ctrlc_tmux.sh, ctrlc_wav.sh, twice.py, termgrp.py). This is outside the repo and may not survive a reboot.

## Pass
- G5: test suite, 18 passed.
- N1 mostly fixed: real Ctrl-C in tmux at 0.3–6 s gives RC 130, no traceback, nothing played. SIGINT sent to the process only, including during `import piper`, is clean.
- G1: Ctrl-C during playback gives 130, clean.
- G2: SIGTERM, to the process or the whole group, during synthesis or playback, gives 143, clean.
- G3: normal speaking works (card1 only, mixer unchanged).
- G4: error exit codes unchanged.
- X2: pressing Ctrl-C repeatedly never leaves a traceback or leftovers.
- No regressions.

## Fail N1b: a stop right after synthesis is swallowed (pre-existing; base 0e4f43e behaves the same)
Steps:
1. In tmux, run `TMPDIR=<dir> say.py 'Willkommen im Automuseum. Hier sehen Sie die Geschichte des Automobils.'; echo RC=$?`.
2. Press Ctrl-C as soon as `<dir>/tmp*/say.wav` appears.

Expected: RC 130, nothing played.

Observed: "Exception ignored in: <function Wave_write.__del__> ... KeyboardInterrupt" is printed, the whole sentence is spoken, and RC is 0. This happened in 7 of 10 SIGINT runs and 3 of 3 SIGTERM runs sent 0.002–0.08 s after say.wav appeared.

Cause (inferred): the handler's exception is raised inside a `__del__`, where Python discards it. `stopped_by` records the signal, but main() checks it only in the `except` path, so speak() goes on to play and returns 0.

Likely fix: check whether a stop was recorded before playing and before returning. For example, raise or return 128+signum when `stopped_by` is non-empty after synthesize(), and in main() after speak() returns.

Variant seen once: a signal at the moment say.wav is created gave RC 130 but printed "Exception ignored ... AttributeError: 'Wave_write' object has no attribute '_file'".

## Concerns
- Ctrl-C in the first 0.02–0.15 s, before main() installs its handler, prints a traceback (rc -2 or 1).
- A stop during synthesis can take up to 7.7 s to take effect, because it is handled only once the onnxruntime call returns.
- Under load (load average 5.4), one call took about 24 s.
- Still present: aplay's own "Interrupted system call" line (C11) and onnxruntime's .ses / mat-debug files in TMPDIR (C12).

## Blocked
- R6 at the CLI needs the USB headset physically unplugged.
