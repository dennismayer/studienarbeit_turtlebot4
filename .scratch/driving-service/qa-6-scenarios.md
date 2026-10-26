# Follow-up QA 6: prior qa-report-5 at a33f82d, now 7b9f919
Fix diff bf59d53..7b9f919: CamelCase modules/entry points; NavigatorNode._drive sees every Nav2 request through (waits for the answer, cancels an acceptance, forwards cancel/Ctrl-C while waiting for the result), _end_unreached (cancel wins); detector MIN_CLUSTER_POINTS=3 and range_max; invalid params -> nonzero exit.
Sandbox: env -u ROS_DISCOVERY_SERVER ROS_DOMAIN_ID=86 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
## Never and always
- Never: a Nav2 goal left driving after Ctrl-C/cancel (within the response timeout); a client left without a result; a goal outside the sandbox.
- Always: a cancel requested before Ctrl-C ends as "Canceled"; Navigator exits under ros2 launch's 10 s SIGKILL; invalid params exit nonzero with the message.
## Setup
S0  isolation check (+ positive control in the robot env)
B1  rename: `ros2 run driving_service navigatorNode` / `visitorDetectorNode`, `ros2 launch driving_service driving_service.launch.py`; old names gone
## Fixed findings
AD1  Ctrl-C right as Nav2 accepts (signal at Nav2's GOAL #1, delays 0/0.05/0.1/0.2 s; and at client "accepted" + 0) -> Nav2 goal canceled, client "Navigator shutting down"
AD2a cancel on the last attempt while Nav2 doesn't answer (attempts 1, silent) -> "Canceled" (after the response timeout)
AD2b cancel on the last attempt while Nav2 rejects late (attempts 1, latereject:2) -> "Canceled"
AD2c cancel on attempt 1 of 3 while Nav2 doesn't answer -> "Canceled", no attempt 2
ST4a cancel mid-drive with Nav2 rejecting the cancel, then Ctrl-C -> "Canceled"
ST4b cancel while Nav2 hasn't answered (late:3), then Ctrl-C -> "Canceled", acceptance canceled
## Reruns (shutdown/cancel paths) + exit time
F1a Ctrl-C, Nav2 absent | F1b Ctrl-C, Nav2 silent, timeout 8 | C22d Ctrl-C, accept 1 s late | T1 SIGTERM mid-drive | I1 SIGINT-ignored start | F2a Ctrl-C mid-drive + next goal at once | X_late45
N6  cancel mid-drive -> Nav2 sees cancel (latency), "Canceled"; next goal served
N7  cancel during retry delay -> "Canceled", no new Nav2 goal
N8c cancel before Nav2 accepted (late:2) -> acceptance canceled, "Canceled"
G3  launch + drive + Ctrl-C -> Nav2 canceled, both nodes finish cleanly
## Detector changes
V14 2-point cluster never a candidate, 3-point is (person_width_min 0.005, min_confidence 0)
V15 person beyond the scan's range_max ignored; at exactly range_max detected
P1  invalid params (nav_retry_attempts 0, nav2_response_timeout 0, stability_threshold 2, min_confidence 1.5, distance min>=max, wrong type) -> nonzero exit + message; boundary valid values start
## Regression flow
F1  launch + main-loop stand-in + scan player + fake Nav2: two Visits end to end, then Ctrl-C
V1  detector on the real /scan read-only (robot env, action_name qa6_*), if the LiDAR publishes
## Exploratory: Nav2 never answers a cancel mid-drive on Ctrl-C (exit time, client result); cancel twice; Ctrl-C twice
## Carried forward from report 5 (a33f82d) and earlier: X2, X2v, N11, N15, V9a-c, R18a-d, N10, X_late3, double Ctrl-C, D1, launch SIGTERM concern

## Results (HEAD 7b9f919, 2026-10-26 07:08-07:45; logs in scratchpad qa6/logs)
PASS S0 (sandbox sees only /rosout, /parameter_events; robot env as super client lists turtlebot4_node etc.) | B1 | P1 (all invalid values rc=255 + message; boundary values start)
PASS AD1 x6: Ctrl-C inside/0/0.05/0.1/0.2 s after Nav2 accepts, and at the client's acceptance -> Nav2 cancel 0.25-0.31 s after Ctrl-C, client "Navigator shutting down", exit 1.2-1.4 s
PASS AD2a/c "Canceled" 4.1 s after the cancel (waits out the 5 s response timeout) | AD2b 1.3 s | AD2d (attempt 3/3, late reject) 1.3 s, attempts=3
PASS ST4b cancel while pending, Ctrl-C, late accept -> Nav2 canceled 0.26 s after accepting, "Canceled", exit 3.0 s
FAIL ST4a/ST4c cancel mid-drive, Nav2 refuses / never answers the cancel, then Ctrl-C -> no result for the client (timeout rc=124), traceback, exit 8.3-8.5 s (base bf59d53: same no-result, exit 3.4 s)
PASS F1a 1.27 s (result 0.10 s after Ctrl-C) | F1b 6.46 s (result 5.2 s after Ctrl-C; QA5 1 s) | C22d 1.68 s | C22b 1.28 s | T1 1.66 s | I1 1.59 s | X_late45 5.58 s (canceled 0.44 s after accept; result 4.4 s after Ctrl-C)
PASS F2a (2nd run) 1.28 s, next goal rejected, one Nav2 goal. 1st run: the stand-in's MoveToLocation request never reached the Navigator (stand-in sends without wait_for_server) - harness, not reproduced
PASS N6 cancel latency 0.14 s, result 0.26 s, next goal Reached | N6cli `ros2 action send_goal` + Ctrl-C -> Nav2 cancel 0.46 s, next CLI goal Reached | N7 Canceled 0.04 s, no 2nd Nav2 goal | N8c late accept canceled 0.24 s after it, "Canceled", next goal Reached
PASS G3 launch Ctrl-C mid-drive 1.54 s, both "finished cleanly" | F1 two Visits under launch, Ctrl-C in visit 2: canceled, next goal rejected, 1.63 s
PASS V14 2-point cluster never reported, 3-point reported | V15 person at 2.0 m ignored with range_max 1.9, reported with 2.0 and 12
FAIL X_ign/X_hang/X_dead Ctrl-C mid-drive while Nav2 ignores, never answers the cancel, or has died -> client never gets a result, traceback, exit 8.3-8.5 s; under launch (G3x_ign, G3x_dead) escalates to SIGTERM at 5 s, exits 8.3-8.4 s. Base: same no-result, exit 3.6/5.6 s
FAIL X_late49i acceptance 4.9 s late + cancel refused, Ctrl-C -> client no result, exit 8.47 s. Base (bf59d53): "Navigator shutting down" 1.0 s after Ctrl-C, exit 5.88 s -> introduced
BLOCKED V1 real /scan: rplidar_composition runs but /scan has 0 publishers (07:15)
NOT RUN: cancel twice, Ctrl-C twice (run_node unchanged; double Ctrl-C carried forward from report 4)
