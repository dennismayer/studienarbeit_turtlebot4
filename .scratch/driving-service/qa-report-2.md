# QA report 2 (follow-up) — driving packages

Tested commit: 79244a7 (fixes since 60dac1c), prior report `qa-report-1.md`. Scenario list: `qa-2-scenarios.md`. Isolation as in report 1 (re-checked, S0 pass).

## Verdict: Fails

All earlier fails and concern N13 pass now, and so do requirements 19–22. One new fail comes from the fix itself, in both nodes: after a peer is killed with -9 and restarted at once, the dead copy stays in the ROS graph for about 20 s. The identity snapshot taken at goal time includes it, so when it drops out the "server changed" check fires although the live server still runs the goal: the goal is dropped without a cancel and a second one is started.

## Reruns of earlier fails — all pass

N11 (Nav2 dies mid-goal: "Nav2 lost the goal", retry, attempts=3 after 25 s; next goal reached), N15 (6 rounds kill -9 + restart + goal 1.5 s later, never stuck; lost request → reached on attempt 2 after ~10.6 s), V9a/b/c (receptionist exits / Ctrl-C'd / killed 2 s before detection: re-arms, new receptionist gets a goal), X1a (Canceled after 0.04 s), N9 and V13 (every invalid value stops the node at startup with a clear message, exit 255; edge values accepted), N13 (Ctrl-C during a drive cancels the Nav2 goal, also under `ros2 launch`).

## New requirements — pass

R18a (silent Nav2: one failed attempt per 5 s, "Nav2 did not answer the goal request", attempts=3), R18b (Nav2 accepting 7 s late is canceled 13 ms after accepting, attempt 2 reaches the goal), R18c/R18d (same for the receptionist), N10/req 21 (NaN/±inf in x, y or yaw rejected; finite extremes accepted).

## Affected behaviour reruns — pass

N1, N2, N3/N5, N4, N6, N7, N8, N12; V2, V6, V7, V10, V8; G3 (both new timeouts load as 5.0, clean Ctrl-C); F1 full flow incl. Ctrl-C during the third Visit's drive cancelling the Nav2 goal.

## V1 real LiDAR — pass, limited

LiDAR up at 18:58:34, 7.5 Hz; TF yaw +90° so the window is base_link ±30° straight ahead; BEST_EFFORT subscription matched with the RELIABLE publisher. No detection over ~18 min with a server and ~13 min without. Limit: the whole front window reads 0.32–0.42 m (something stands right in front of the robot), closer than `detection_distance_min`, so "no false detection" is weak.

**Step for the user:** clear the area in front of the robot, then have a person stand ~1.5 m straight ahead, then ~20° to the left. Each should give one goal with angle ≈ 0 and ≈ +0.35 rad.

## New fails

- **X2 — Navigator drives a goal twice on a restarted Nav2.** Fake Nav2 A running; kill -9 A and start B at once; 3 s later send a goal (B takes 40 s). ~20 s after the kill: "Attempt 1/3: Nav2 lost the goal" while B still ran it; no cancel to B; attempt 2 sent, B ran two goals at once, client feedback alternated attempt 1/2; Reached, attempts=2 after 60 s. Same with a 15 s drive, and spontaneously in the N11 rerun.
- **X2v — Detector sends a second VisitorDetected goal while one is open** (against req 15/18). Receptionist A; kill -9 A, start B (holds visits 45 s); visitor 3 s; 25 s later another visitor. At ~+20 s "its visit is lost" while B still ran visit 1; goal #2 sent to B; visit 1 not canceled.

## Concerns

- Nav2 dies and nothing replaces it: the final message is "Nav2 did not answer the goal request", not "Nav2Unavailable" (the dead server still looks ready until it leaves the graph).
- Ctrl-C of the Navigator still prints a RuntimeError traceback and "Goal state not set"; the MoveToLocation client never gets a result and stops only at its own timeout.
- A slow but alive receptionist gets a new request about every 6 s, each late acceptance canceled.

## Not covered

Real Nav2 (localization message), a real person in front of the LiDAR (user step above), `colcon test`.
