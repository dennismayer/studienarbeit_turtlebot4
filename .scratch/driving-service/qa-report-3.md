# QA report 3 (follow-up) — driving packages

Tested commit: 98072da (fixes since 79244a7), prior report `qa-report-2.md`. Scenario list: `qa-3-scenarios.md`. Isolation re-checked (S0 pass).

## Verdict: Fails

X2 and X2v are fixed; N11, N15, V9a/b/c still pass. Requirement 22 fails on two Navigator shutdown paths.

## Fails

1. **Ctrl-C while the Navigator waits — the client gets no result.** No Nav2 running, send a MoveToLocation goal, Ctrl-C 1.5 s later: the node exits, the client never gets a result (waits until its own 40 s timeout); log "Failed to check if action server is available: rcl node's context is invalid", traceback, "Goal state not set". Same with a Nav2 that never answers and `nav2_response_timeout:=8.0` ("RuntimeError: Shutting down"). With the default 5 s timeout it only just passes (result 4.6 s after Ctrl-C). Likely cause: `_wait_for_nav2` and `_wait(send_future, …)` don't check `_stopping`; `stop()` waits ~4.5 s in total.
2. **Ctrl-C during a drive while the client sends its next goal — a new Nav2 goal goes out and Nav2 drives on with nobody owning it.** Fake Nav2 with 10 s drives, main-loop stand-in, navigatorNode; Ctrl-C during the first drive: first Nav2 goal canceled and "Navigator shutting down" delivered, then the main loop's next MoveToLocation is accepted during shutdown, Nav2 goal #2 is sent, the Navigator exits with "RuntimeError: Shutting down", and goal #2 runs to SUCCEED 10 s later with no cancel. Seen first in F1 under `ros2 launch`. Likely cause: `_on_goal` doesn't reject while `_stopping`, and `stop()` keeps spinning (busy wait + 0.5 s flush).

## Passes

X2 (40 s and 15 s: Reached, attempts=1, one Nav2 goal), X2v (no second goal while visit 1 open), C22a (Ctrl-C during a drive: canceled, client gets "Navigator shutting down" 0.5 s later, no traceback), C22b (Ctrl-C during retry delay: no new Nav2 goal, result delivered), C22d (Ctrl-C while Nav2 accepts 1 s late: late acceptance canceled, result delivered), N11, N15, V9a/b/c, F1 (two Visits, operator goal rejected, yaml loaded; Ctrl-C part is fail 2). Carried forward from report 2: N1–N10, N12, R18a–d, V2–V8, V10, V13, N9, G3, V1 (limited).

## Concerns

- **SIGTERM** during a drive kills the Navigator at once: no stop(), Nav2 goal not canceled, Nav2 ran on to SUCCEED. `ros2 launch` escalates to SIGTERM; systemd stops with SIGTERM.
- **SIGINT ignored at start** (e.g. started with `&` from a script): with rclpy's handler off, such a Navigator can't be stopped with Ctrl-C any more; needed SIGTERM.
- After a receptionist crash and immediate respawn, a visitor in the next ~20 s isn't served (old visit counts as open until the dead copy leaves the graph) — follows requirement 18.

## Not covered

Real Nav2, a real person in front of the LiDAR, `colcon test`.
