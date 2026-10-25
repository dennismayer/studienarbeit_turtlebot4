# QA report 1 — driving packages

Tested commit: 60dac1c (fixed point 371feb3). Spec: `spec.md` in this folder. Full re-run after the power loss; everything that served or sent navigation goals ran isolated (`env -u ROS_DISCOVERY_SERVER ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`). Scenario list: `qa-1-scenarios.md`. Raw logs were in the session scratchpad (`qa1/logs/`) and are not kept.

## Verdict: Fails

Common cause of the three fails: nothing times out a lost action response. When the peer dies or a request is lost, the node waits forever and is unusable until restarted. Launch respawn doesn't help because the node doesn't crash.

## Fails

- **N11 — Nav2 dies during a goal, the Navigator is stuck for good.** Fake Nav2 exits during execute; cancel after 15 s is accepted but no result follows; after a new working fake Nav2 comes up, the next goal is rejected "already driving".
- **N15 — goal sent shortly after a Nav2 restart is lost, Navigator stuck for good.** kill -9 fake Nav2, start a new one, send a goal 1.5 s later; hung on the 3rd try: goal accepted, Nav2 never got the request, all later goals rejected "already driving". Also seen spontaneously in N8a and X1d (~3–4 s after a restart). Blocks in `navigator_node.py:101` `self._wait(self._nav2.send_goal_async(...))`.
- **V9 — receptionist dies/stops during a Visit, the detector never re-arms** (contradicts req 16 "server error", req 17).
  - (a) server exits during the Visit; a new server comes up; visitor in front 15 s + 8 s: no goal.
  - (b) server Ctrl-C'd during the Visit: same.
  - (c) server killed 2 s before a detection: detector still sees it as ready, sends the goal into the void, no warning, dead for good.
- **X1a (minor)** — cancel while the Navigator waits for an absent Nav2 is accepted, but the result is `Nav2Unavailable` after 5 s, not "Canceled" (req 9).
- **N9 (minor)** — `nav_retry_attempts` 0 or −2: every goal aborts at once with `message=''`, `attempts=0`, nothing logged.
- **V13 (minor)** — `stability_threshold` 2 or 0: never reports anyone; `lidar_frame_skip_rate` 0: crashes on the first scan (modulo by zero). No startup validation.

## Passes

S0 sandbox isolation; G1 layout/build; G2 label; G3 launch (params, clean Ctrl-C, respawn); I1/I2 interfaces; N1 Nav2Unavailable; N2 reached + feedback + map pose/yaw; N3 retries with 5 s gaps; N4 failure message; N5 busy reject; N6 cancel forwarded; N7 cancel during retry delay; N8 Nav2 rejection retried; N10 yaw wrap; N12 Nav2 up 2 s after goal; N14 no /initialpose; V2 one goal per visit; V3 angles/window/closest of two; V4 distance/width bounds; V5 stability and skip rate; V6 re-arm timing with defaults; V7/V10 reject/abort/crash with server staying up; V8 server cleanly absent (throttled warning, no queueing); V11 NaN/inf/below range_min ignored; V12 detector never cancels; /scan BEST_EFFORT; seam at lidar_front_angle = π; F1 full flow with a stand-in main loop (two Visits, operator goal during a Visit rejected).

## Blocked

- **V1 detector on the real LiDAR** — `/scan` not published after the reboot (journal: "RPLIDAR stopped" from turtlebot4_node at 16:08:20). Seen: detector starts/stops cleanly in the robot env; TF base_link→rplidar_link yaw +90° confirms the −π/2 default. Needs the LiDAR running.

## Concerns

- **N13** — Ctrl-C (or crash) of the Navigator during a drive doesn't cancel the Nav2 goal; the fake Nav2 ran on to SUCCEED 12 s later. On the robot Nav2 would keep driving with nobody owning the goal. Shutdown also prints a RuntimeError traceback and "Goal state not set".
- Detector logs "Visit over: " with an empty message after an abort/crash and never the status; prints "-0 deg".
- NaN/inf goal coordinates are passed to Nav2 unchanged.
- `ros2 action send_goal` + Ctrl-C prints "Executor is already spinning" (ros2cli quirk; cancel still reached Nav2).

## Not covered

Real Nav2 without localization (req 11's actual message) — needs a real Nav2, not run for safety. `colcon test` not run (QA of running nodes).
