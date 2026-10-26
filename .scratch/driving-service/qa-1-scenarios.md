# QA scenarios: driving_interfaces + driving_service (371feb3..60dac1c)
Re-run after the reboot: every scenario below is re-observed at HEAD 60dac1c (pre-reboot results discarded).
Sandbox = env -u ROS_DISCOVERY_SERVER ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST (envsb.sh).

## Never and always
- Never: a MoveToLocation / NavigateToPose goal outside the sandbox; nothing published to robot topics.
- Never: more than one open VisitorDetected goal; the detector cancelling a goal.
- Never: Navigator stuck busy after a goal ends; a traceback in normal use.
- Always: Ctrl-C stops nodes cleanly; launch starts both nodes with yaml params.

## Scenarios (status filled in below as observed)
S0 sandbox isolation | G1 layout/build | G2 labels | G3 launch | I1 MoveToLocation | I2 VisitorDetected
N1 Nav2 unavailable | N2 success+feedback+pose | N3 fail,fail,succeed | N4 always fail | N5 concurrent goal
N6 cancel during drive | N7 cancel during retry delay | N8 Nav2 rejects | N9 retry params 0/1/-2
N10 odd goal values | N11 Nav2 dies mid-goal | N12 Nav2 late | N13 Ctrl-C mid-goal | N14 no initial pose
V1 real /scan | V2 visitor ahead | V3 angle/window | V4 bounds | V5 stability/skip | V6 re-arm timing
V7 rejected | V8 no server | V9 server dies mid-visit | V10 abort/raise | V11 invalid ranges | V12 never cancels
V13 param data attacks | F1 launch flow | R1 regression | X1 exploratory

## Results (HEAD 60dac1c, observed 2026-10-25 16:14-16:51)
PASS: S0 G1 G2 G3 I1 I2 N1 N2 N3 N4 N5 N6 N7 N8 N10 N12 N14 V2 V3 V4 V5 V6 V7 V8 V10 V11 V12 F1
FAIL: N11 (Nav2 dies mid-goal -> goal never ends, cancel ignored, Navigator busy forever)
FAIL: N15 (goal sent shortly after a Nav2 restart -> request lost, Navigator busy forever; 1 of 3 tries + X1d)
FAIL: V9 (receptionist dies or is Ctrl-C'd mid-visit -> detector never re-arms; also V8b: server killed just before a detection)
FAIL(minor): X1a (cancel while waiting for Nav2 -> result Nav2Unavailable, not Canceled)
FAIL(minor): N9 (nav_retry_attempts 0 / -2 -> aborted with empty message, nothing logged)
FAIL(minor): V13 (stability_threshold 0/2 -> detector silently never reports; lidar_frame_skip_rate 0 -> crash on first scan)
CONCERN: N13 (Ctrl-C of Navigator mid-drive leaves the Nav2 goal running + traceback)
BLOCKED: V1 (real /scan not published: RPLIDAR stopped by turtlebot4_node at 16:08:20, after the reboot)
