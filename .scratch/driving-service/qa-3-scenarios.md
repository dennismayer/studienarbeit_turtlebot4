# Follow-up QA 3: driving_service, prior qa-report-2 at 79244a7, now 98072da
Fix diff 79244a7..98072da: action_peer.server_gone, navigator stop()/_stopping, detector uses server_gone.
## Never and always
- Never: nav goal outside sandbox; a second Nav2 goal for one attempt; a second VisitorDetected goal while one is open; a Nav2 goal left running after Ctrl-C; a new Nav2 goal after Ctrl-C.
- Always: Ctrl-C ends the MoveToLocation goal as aborted "Navigator shutting down"; nodes exit cleanly.
## Reruns of report-2 fails
X2   kill -9 Nav2 A, start B at once, goal 3 s later, B 40 s and 15 s -> reached once, attempts=1, one Nav2 goal
X2v  kill -9 receptionist A, start B (45 s visits), visitor 3 s, another 25 s later -> no 2nd goal while visit 1 open
## Report-2 concern -> req 22
C22a Ctrl-C during drive -> Nav2 goal canceled, client result aborted "Navigator shutting down"
C22b Ctrl-C during retry delay -> no new Nav2 goal, client aborted "Navigator shutting down"
C22c Ctrl-C while waiting for absent Nav2; C22d Ctrl-C while Nav2 hasn't answered the goal request (exploratory)
## Affected reruns
N11 (Nav2 dies mid-goal, then back), N15 (restart loop), V9a/b/c, F1 (launch flow + Ctrl-C under launch)
## Carried forward (report 2 @79244a7): everything else (N1-N10, N12, R18a-d, V2-V8, V10, V13, N9, G3, V1-limited)

## Results (HEAD 98072da, 2026-10-25 20:23-20:50)
PASS: S0, X2 (40 s and 15 s: reached once, attempts=1, one Nav2 goal), X2v (no 2nd goal while visit open)
PASS: C22a Ctrl-C during drive (Nav2 canceled, client aborted "Navigator shutting down", no traceback)
PASS: C22b Ctrl-C during retry delay (no new Nav2 goal, client aborted "Navigator shutting down")
PASS: C22d-late Ctrl-C while Nav2 accepts 1 s later (canceled, client "Navigator shutting down"); C22d-silent default 5 s timeout (client "Navigator shutting down", 4.6 s after Ctrl-C)
PASS: N11, N15, V9a, V9b, V9c
FAIL: C22c Ctrl-C while waiting for an absent Nav2 -> client gets no result; traceback (C_nonav2b_*); same with silent Nav2 + nav2_response_timeout 8 (C_silent8_*)
FAIL: C22e / F1 Ctrl-C mid-drive while the client sends its next goal -> new goal accepted during shutdown, new Nav2 goal sent, Navigator exits, Nav2 drives it uncanceled (F1_*, C22e_*)
CONCERN: SIGTERM mid-drive kills the Navigator at once, Nav2 goal not canceled (T_*)
CONCERN: harness note - a node started with SIGINT ignored (inherited) can no longer be stopped by SIGINT (rclpy handler is off) (C_nonav2_*)
CONCERN: after a receptionist crash + respawn, visitors in the next ~20 s are not served (V9a)
