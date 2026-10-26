# QA report 4 (follow-up) — driving packages

Tested commit: 79fb9b9 (fixes since 98072da), prior report `qa-report-3.md`. Scenario list: `qa-4-scenarios.md`. Isolation re-checked (S0 pass).

## Verdict: Fails, on one narrow edge case

Both report-3 fails and both report-3 concerns are fixed; every Ctrl-C/SIGTERM path the diff touches passes.

## Passes

F1a (Ctrl-C with Nav2 absent: "Navigator shutting down" 0.3 s later, clean exit), F1b (silent Nav2, timeout 8 s: result ~1 s after Ctrl-C), F2a/C22e (Ctrl-C during a drive while the main loop sends its next goal: Nav2 goal canceled, result delivered, next goal rejected, no second Nav2 goal), F2b (same under `ros2 launch`: two Visits, Ctrl-C in the third drive, 5 Nav2 goals in total, both nodes finish cleanly), T1 (SIGTERM during a drive: canceled, result, exit 1.2 s), I1 (started with SIGINT ignored: Ctrl-C works), C22a/N13, C22b, C22d, G3, D1 (VisitorDetector SIGINT/SIGTERM idle and during a visit, also with SIGINT ignored at start: exits in ~0.5–0.8 s, no cancel of the receptionist's goal), double Ctrl-C, Nav2 accepting 2.5 s after Ctrl-C (canceled). Carried forward from report 3: X2, X2v, N11, N15, V9a/b/c, visit flow.

## Fail (edge, exploratory)

**X_late45 — a Nav2 acceptance that arrives after the Navigator exited is never canceled.** Fake Nav2 accepts 4.5 s after the request, then drives 30 s; Ctrl-C 0.5 s after Nav2 got the request: client gets "Navigator shutting down", the Navigator exits 3.2 s after Ctrl-C, Nav2 accepts ~0.8 s later and drives to SUCCEED with no cancel. Window: acceptance later than `stop()`'s ~2 s wait on the pending request (≈3.2 s after the signal) but before `nav2_response_timeout` (5 s). A real Nav2 normally answers within milliseconds.

## Concerns

- SIGTERM to the `ros2 launch` process itself killed only launch; both nodes ran on as orphans (ros2 launch behaviour, not this package's). systemd stopping the whole service signals every process.
- SIGINT during Python import (~0.3 s after start) prints a KeyboardInterrupt traceback; harmless, before any ROS activity.

## Not covered

Real Nav2, a real person in front of the LiDAR (user step), systemd-managed shutdown, `colcon test`.
