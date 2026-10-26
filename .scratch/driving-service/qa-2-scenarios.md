# Follow-up QA: driving_service, prior report qa-report-1 at 60dac1c, now 79244a7
Fix diff: 60dac1c..HEAD (navigator_node, run_node, visitor_detector(_node), action_peer, ros_params.yaml).
Sandbox for everything navigation: env -u ROS_DISCOVERY_SERVER ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

## Never and always
- Never: a nav goal outside the sandbox; a node stuck forever; two open VisitorDetected goals; a Nav2 goal left running nobody owns.
- Always: Ctrl-C ends nodes cleanly; launch starts both nodes with yaml params.

## Reruns (earlier fails / concern)
N11  Nav2 dies mid-goal -> attempt fails, retry/Nav2Unavailable, cancel works, next goal served (req 18)
N15  kill -9 + restart Nav2, goal 1.5 s later, x6 -> never stuck (req 18)
V9a  receptionist exits mid-visit -> re-arm, new receptionist gets next visitor (req 18)
V9b  receptionist Ctrl-C'd mid-visit -> same
V9c  receptionist killed 2 s before a detection -> no permanent stall (req 17/18)
X1a  cancel while waiting for absent Nav2 -> Canceled (req 19)
N9   nav_retry_attempts 0/-2 (+ other Navigator params) -> startup error (req 20)
V13  stability_threshold 2/0, skip 0 (+ other detector params) -> startup error (req 20)
N13  Ctrl-C Navigator mid-drive -> Nav2 goal canceled (req 22)
## New requirements
R18a Nav2 never answers goal request -> failed attempt after nav2_response_timeout; R18b late acceptance canceled
R18c receptionist never answers -> visit not started after receptionist_response_timeout; R18d late acceptance canceled
N10  NaN / inf goals rejected (req 21)
## Affected by the fix (reruns)
N1 N2 N3 N4 N5 N6 N7 N8 N12 (navigator _drive rewritten), V2 V6 V7 V8 V10 (detector node changed), G3 launch Ctrl-C/respawn
F1   adjacent regression flow: launch + main-loop stand-in + fake Nav2, two Visits
X2   exploratory: server identity check with a ghost (killed -9) server still in the graph
## Carried forward from report 1 (60dac1c, unaffected): S0* G1 G2 I1 I2 N14 V3 V4 V5 V11 V12
## Blocked: V1 (real /scan) - re-checked at 18:57, still not published

## Results (HEAD 79244a7, observed 2026-10-25 18:57-19:33)
PASS reruns: N11 N15 V9a V9b V9c X1a(=req19) N9(=req20) V13(=req20) N13(=req22, also under launch)
PASS new: R18a R18b R18c R18d N10(=req21)
PASS affected: N1 N2 N3 N4 N5 N6 N7 N8 N12 V2 V6 V7 V8 V10 G3 F1 S0
PASS (limited) V1: real /scan, TF yaw +90 => window = base -30..+30 deg; ~30 min, no false detection; front window occluded at 0.32-0.42 m
FAIL X2  Navigator: Nav2 kill -9 + immediate restart, goal within ~20 s -> "Nav2 lost the goal" at ghost expiry while the SAME server still runs it; not canceled; retry runs concurrently; double drive (X2_*, X2b_*)
FAIL X2v Detector: receptionist kill -9 + immediate restart, visitor within ~20 s -> "visit is lost" at ghost expiry while B still runs it; second VisitorDetected goal sent while first open (X2v_*)
CONCERN: N11 final message "Nav2 did not answer" instead of Nav2Unavailable when Nav2 died (ghost); Ctrl-C traceback still printed; MoveToLocation client gets no result on Navigator Ctrl-C
