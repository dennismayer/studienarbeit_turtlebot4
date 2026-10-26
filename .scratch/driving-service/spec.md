# Spec: driving_interfaces + driving_service

Status: in progress
Branch: feat/driving-service (base: main @ 371feb3)

Settled in /grill-with-docs on 2026-10-25. Terms: see `GLOSSARY.md` (Visitor, Visit, Location). Decision record: `docs/adr/0001-visitor-detector-is-the-action-client.md`.

These two packages are agent-owned (see the exception in `AGENTS.md`).

## Requirements

### General
1. Two new packages: `src/driving_interfaces` (ament_cmake, interface definitions only) and `src/driving_service` (ament_python), laid out like `speaking_interfaces` / `speaking_service`.
2. Every package carries a label that it was created with Claude on the basis of the car_museum_guide package. That's the only place that package is mentioned; the code is about the receptionist bot only.
3. ROS 2 Jazzy, system Python. Parameters in `config/ros_params.yaml`, both nodes started by `launch/driving_service.launch.py`.

### driving_interfaces
4. `action/MoveToLocation.action`:
   - Goal: `float64 x`, `float64 y` (map frame, m), `float64 yaw` (rad, 0 = map x axis, CCW positive).
   - Result: `bool success`, `string message` (`"Reached"`, `"Canceled"`, `"Nav2Unavailable"` or the failure description), `uint8 attempts`.
   - Feedback: `uint8 attempt` (starts at 1), `float32 distance_remaining` (m, from Nav2).
5. `action/VisitorDetected.action`:
   - Goal: `float32 distance` (m, LiDAR to visitor cluster), `float32 angle` (rad, relative to the robot heading, 0 = ahead, left positive), `float32 confidence` (0..1).
   - Result: `bool success`, `string message`.
   - No feedback.

### Navigator node (driving_service) — MoveToLocation server
6. Serves `MoveToLocation` by sending `nav2_msgs/action/NavigateToPose` goals to Nav2 directly (no BasicNavigator).
7. One goal at a time: a goal that arrives while one is running is rejected.
8. When Nav2 fails a goal, it's retried up to `nav_retry_attempts` (default 3) times total, with `nav_retry_delay` (default 5 s) between attempts. Feedback carries the current attempt and Nav2's distance_remaining.
9. Canceling the MoveToLocation goal cancels the running Nav2 goal (and any pending retry); the result is `"Canceled"`.
10. If the Nav2 action server isn't available, the goal fails with `"Nav2Unavailable"` (no retries).
11. The node does not set the initial pose. Localization is done outside (RViz 2D Pose Estimate or AMCL params); goals fail with a clear message until then.

### Visitor detector node (driving_service) — VisitorDetected client
12. Continuously clusters `/scan` (best-effort sensor QoS): valid ranges only, front window ±`detection_angle_range`/2 around `lidar_front_angle` (the scan angle that points straight ahead; default −π/2 because the TurtleBot4 URDF mounts the LiDAR rotated +90°), clusters by gap between neighboring points, min points per cluster.
13. A cluster is a visitor candidate when its mean distance is within [`detection_distance_min`, `detection_distance_max`], its width is within [`person_width_min`, `person_width_max`], and its confidence (weighted width/distance/cluster-size score) is ≥ `min_confidence`.
14. Stable detection: at least `stability_threshold - 2` of the last `stability_threshold` processed scans had a candidate. `lidar_frame_skip_rate` thins processing.
15. On a stable detection while armed, it sends one `VisitorDetected` goal with the best candidate's distance, angle and confidence, and waits for the result. No new goal while one is open; the detector never cancels it.
16. Re-arming after a Visit (result received, goal rejected, or server error): only once no candidate has been seen for `visitor_timeout` seconds, and then after `cooldown_period` more seconds. The same re-arm applies if the goal is rejected.
17. If no VisitorDetected server is available, the detector logs it (throttled) and stays armed; it does not queue detections.

### Robustness (added after QA report 1, 2026-10-25)
18. A peer that dies, restarts or never answers must not leave a node stuck:
    - "Server gone" means every server endpoint that was in the graph when the goal was sent has left it. A crashed copy can linger in the graph for ~20 s after kill -9; it leaving while the live server stays does not count (QA report 2, X2/X2v).
    - Navigator: a Nav2 goal request that gets no answer within `nav2_response_timeout` is a failed attempt (a late acceptance is canceled right away); a Nav2 server that is gone while a goal runs fails that attempt; before each attempt the Nav2 server must be available, else the goal fails with `"Nav2Unavailable"`.
    - Detector: a VisitorDetected goal request with no answer within `receptionist_response_timeout` counts as a visit that never started (a late acceptance is canceled right away); a receptionist that is gone during a Visit ends the Visit (re-arm as in 16).
19. Canceling ends the goal as `"Canceled"` at every stage: also while the Navigator still waits for the Nav2 server or for Nav2's answer, and on the last attempt (code review AD2).
20. Invalid parameters (e.g. `nav_retry_attempts` < 1, `stability_threshold` < 3, `lidar_frame_skip_rate` < 1, min ≥ max ranges, `min_confidence` outside 0..1) stop the node at startup with a clear error.
21. MoveToLocation goals with a NaN or infinite coordinate are rejected.
22. On Ctrl-C/SIGINT the Navigator cancels the running Nav2 goal before it exits (user decision 2026-10-25: Nav2 must not keep driving with nobody owning the goal), sends no new Nav2 goal, rejects goals that arrive during shutdown, and ends the running MoveToLocation goal as aborted with `"Navigator shutting down"` so the client isn't left waiting — also while it still waits for Nav2. A cancel requested before Ctrl-C wins: the goal ends as `"Canceled"` (code review ST4). Ctrl-C in any spin step, also the one that accepts the goal, leaves no Nav2 goal driving (code review AD1). SIGTERM (what `ros2 launch` escalates to, what systemd sends) behaves like SIGINT, and SIGINT works even if it was ignored when the node started (QA report 3).

### Out of scope
- The VisitorDetected server (the receptionist main loop in `receptionist_bot`) — the user writes it.
- Return-to-reception logic and the list of saved Locations — owned by the receptionist.
- Moving the map out of the old package.

## Progress

### Done
- Grilling, glossary, ADR 0001.
- Implemented test-first and mutation-checked (commits af7a08c..71f960b): `driving_interfaces`; `driving_service` with `visitor_detector.py` (pure logic), `navigator_node.py`, `visitor_detector_node.py`, `run_node.py`, `config/ros_params.yaml`, `launch/driving_service.launch.py`.
- Mutation survivor (equivalent, kept): the explicit "rejected goal" branch in `VisitorDetectorNode._on_goal_response`; rclpy also answers a result request for a rejected goal, so the outcome is the same, but relying on that is fragile.

- QA report 1 (at 60dac1c): Fails — see `qa-report-1.md`. Fails N11, N15, V9, X1a, N9, V13 and concern N13 lead to requirements 18–22.
- V1 (detector on the real LiDAR) blocked: the LiDAR stopped after a power loss; the user restarts it (decision 2026-10-25).

- Requirements 18–22 implemented test-first and mutation-checked (8a2ba6c, 9524dd2). Mutation survivor (kept): the "already given up on this visit" guard in `VisitorDetectorNode._on_visit_over`; no fake can deliver a late result from a vanished receptionist.
- Test-harness lessons (rclpy Jazzy): a node added to an already spinning executor is not always picked up; destroying an action server inside a spinning executor can kill the spin thread; `destroy_node()` leaves action servers alive. Fakes now run in executors of their own and stop like a dying process.

- QA report 2 (at 79244a7): Fails — new X2/X2v from the identity check; see `qa-report-2.md`. Fixed with the "server gone" rule above; Ctrl-C now also ends the client's goal (concern from report 2).
- Open concerns from report 2, not changed: a dead Nav2 with no replacement ends as "Nav2 did not answer the goal request" (it still looks ready until it leaves the graph); a slow but alive receptionist gets a new request about every 6 s.

- QA report 3 (at 98072da): Fails — two Ctrl-C paths of requirement 22; see `qa-report-3.md`. Fixed, plus the SIGTERM and ignored-SIGINT concerns. The signal handler only sets a flag: raising into `spin_once` could leave the rclpy executor marked as spinning.
- Open concern from report 3, by design (requirement 18): after a receptionist crash and immediate respawn, a visitor in the next ~20 s isn't served, because the old visit counts as open until the dead copy leaves the graph.

- QA report 4 (at 79fb9b9): Fails on one edge (X_late45): a Nav2 acceptance arriving after the Navigator exited but within `nav2_response_timeout` was never canceled. Fixed: on Ctrl-C the Navigator waits for a pending answer until the response timeout would have run out, at most 5 s (ros2 launch kills after 10 s). Known limit: an acceptance arriving after the Navigator process has exited can't be canceled by it any more — i.e. one later than `nav2_response_timeout` after the request (QA report 5, X_late55), or, with `nav2_response_timeout` set above 5 s, one later than 5 s after Ctrl-C. A real Nav2 answers within milliseconds.
- Survivor (kept): `stop()` reads the pending request before setting the stop flag; the other order is a microsecond race no test can hit reliably.
- Concerns from report 4, outside this package: SIGTERM to the `ros2 launch` process itself kills only launch and leaves the nodes running (stop with Ctrl-C, or signal the nodes / the whole service); SIGINT during Python import prints a harmless traceback.

- QA report 5 (at a33f82d): **Passes** — see `qa-report-5.md`; its one concern (X_late55) is the known limit above.

- Code review (since 371feb3, at bf59d53): **Ready with fixes**. Findings AD1 (P1), AD2, SP1–SP4, ST1–ST9; dispositions below.
- User decision 2026-10-26 on ST1 (snake_case node modules vs CamelCase in speaking_service): **O2, rename the node modules to CamelCase** (`NavigatorNode.py`, `VisitorDetectorNode.py`) to match speaking_service; helper modules stay lowercase like speaking_service's `scripts/cache.py`.
- Review fixes, structural (refactor:, 56e5080..bffbe2f): ST1 CamelCase modules; ST2–ST6 shared `params` / `action_peer` helpers and `VisitorCandidate`; ST3/ST7 shared test tools, no test-only node methods.
- Review fixes, behavior (test-first, mutation-checked): AD1/SP3 and AD2/SP4 and ST4 — `_drive` sees every Nav2 request it sent through to the end (waits for the answer, at most `STOP_PENDING_MAX` after Ctrl-C, and cancels an acceptance; forwards a cancel or Ctrl-C while it waits for the result), and every unreached goal ends in `_end_unreached`, where a cancel wins over Ctrl-C. `stop()` no longer reads a pending request, so its old survivor is gone. SP1 `MIN_CLUSTER_POINTS = 3`; SP2 readings above `range_max` are ignored; ST8 process-level test of invalid parameters. 7 Navigator and 4 detector mutants, all caught by the intended tests.
- Flake fixed: the late-acceptance retry test had 0.5 s of margin; under suite load a starved wait thread took the late answer as in time (da00389).

### Next steps
- Fix the review findings (structural first as refactor:, then behavior test-first), follow-up QA and review.

### How to run
- Gate: `colcon build --packages-select driving_interfaces driving_service && colcon test --packages-select driving_service && colcon test-result --test-result-base build/driving_service`
- ROS tests run in ROS domain 87 with the discovery server disabled (`ROS_DISCOVERY_SERVER` unset, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) and unique names, so they never reach the robot's real Nav2. The domain ID alone does not isolate on this robot: its Fast DDS discovery server (`127.0.0.1:11811`) connects participants across domains (verified 2026-10-25). Do the same for any manual experiment: `env -u ROS_DISCOVERY_SERVER ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ...`.
- Launch: `ros2 launch driving_service driving_service.launch.py`
