# QA report 6 (follow-up) — driving packages

Tested commit: 7b9f919 (code changes since bf59d53), prior report `qa-report-5.md` (a33f82d). Scenario list: `qa-6-scenarios.md`. Sandbox: domain 86, no discovery server, localhost only. S0 passed: the sandbox saw only `/rosout` and `/parameter_events`, and the robot environment (as super client) listed the robot's nodes. Nav2 was not running on the robot, and no goal left the sandbox.

## Verdict: Fails

The fixed findings work: AD1, AD2 and ST4 (cancel while Nav2 hasn't answered yet) pass, and so does every rerun with a Nav2 that cancels when told. It fails when Ctrl-C reaches a running Nav2 goal that Nav2 never ends: the client gets no result. For mid-drive goals the base build did the same. For a late acceptance the failure is new. Exit time also grew to about 8.5 s.

## Fails

**X_ign / X_hang / X_dead / ST4a / ST4c**: requirement 22 says "so the client isn't left waiting".
Steps: fake Nav2 accepts and drives. Then either it rejects the cancel (`--ignore-cancel`), never answers it (`--hang-cancel`), or the process dies mid-goal (`die`, the realistic case). Ctrl-C the Navigator 2 s into the drive. Optionally the client cancels first (ST4a/c).
Observed: the client never gets a result (its 40 s timeout ends it, rc=124). Log shows `Error raised in execute callback: Shutting down` with a traceback and `Goal state not set, assuming aborted`. Exit 8.3–8.5 s after Ctrl-C. Under `ros2 launch` (G3x_ign, G3x_dead): `failed to terminate '5' seconds after receiving 'SIGINT', escalating to 'SIGTERM'`, exit 8.3–8.4 s.
Expected: aborted "Navigator shutting down", or "Canceled" after a cancel (ST4), within a few seconds.
Base bf59d53: also no result (pre-existing), exit 3.4–5.6 s.
Where: `_wait_for_result` gives up only when the Nav2 server leaves the graph. After the cancel is forwarded nothing else ends the wait, so `_end_unreached` never runs. `stop()` waits `STOP_PENDING_MAX + STOP_CANCEL_TIMEOUT` (7 s), then the shutdown raises inside `_wait`.

**X_late49i** (introduced): Nav2 accepts 4.9 s late and rejects the cancel. Ctrl-C 0.3 s after the request. Observed: no client result, exit 8.47 s. Base: "Navigator shutting down" 1.0 s after Ctrl-C, exit 5.88 s.

## Passes (exit time after the signal)

AD1 ×6 (Ctrl-C at or within 0.2 s of acceptance, and at the client's acceptance) 1.2–1.4 s; Nav2 sees the cancel 0.25–0.31 s after Ctrl-C. AD2a/b/c/d all end as "Canceled", also on attempt 3/3. ST4b 3.0 s. F1a 1.27 s. F1b 6.46 s. C22d 1.68 s. C22b 1.28 s. T1 1.66 s. I1 1.59 s. F2a 1.28 s. X_late45 5.58 s.
N6: cancel latency 0.14 s, result 0.26 s. N6 through `ros2 action send_goal` + Ctrl-C: 0.46 s. N7 0.04 s. N8c: acceptance canceled 0.24 s after it arrived.
G3 1.54 s. F1 (two Visits under launch, then Ctrl-C) 1.63 s.
V14 (2-point cluster ignored, 3-point reported), V15 (range_max), P1 (invalid parameters: rc 255 plus the message), B1 (rename).

## Concerns

- Client result after Ctrl-C now waits for Nav2's answer: F1b 5.2 s (QA5 1 s), X_late45 4.4 s.
- A cancel while Nav2 is silent ends only after `nav2_response_timeout` (4.1 s).
- F2a run 1: the stand-in's MoveToLocation request never reached the Navigator. The stand-in sends without `wait_for_server`; the rerun passed.
- `install/` still holds the stale `navigator_node.py` and `visitor_detector_node.py`.
- Stray `exception was never retrieved: cannot use Destroyable` when the detector shuts down.
- Leftover: a `ros2 daemon` for domain 0 started by S0 (pid 131495, not a super client). Stopping it was denied. Remove it with `kill 131495`.

## Not covered

V1 real LiDAR: blocked, `/scan` has 0 publishers. Not run: real Nav2, cancel twice, Ctrl-C twice, systemd, `colcon test`.
