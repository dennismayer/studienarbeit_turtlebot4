# Follow-up QA 7: prior qa-report-6 at 7b9f919, now cde8436
Fix diff: after Ctrl-C, `_wait_for_result` gives up STOP_CANCEL_TIMEOUT (2 s) after the forwarded cancel; stop() busy wait = Ctrl-C + 5 + 2 + 1 s.
Sandbox: env -u ROS_DISCOVERY_SERVER ROS_DOMAIN_ID=86 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST (clean rebuild by the coordinator)
## Never and always
- Never: a client without a result after Ctrl-C; SIGKILL under ros2 launch; a traceback on shutdown; a Nav2 goal left driving that Nav2 would have canceled.
- Always: a cancel before Ctrl-C ends as "Canceled"; without Ctrl-C a refused cancel keeps the goal running.
## Reruns of report-6 fails (plain ros2 run): result + exit time
X_ign  Ctrl-C mid-drive, Nav2 rejects the cancel -> "Navigator shutting down" ~2 s after Ctrl-C
X_hang Ctrl-C mid-drive, Nav2 never answers the cancel -> same
X_dead Nav2 process dies mid-goal, Ctrl-C 2.5 s later -> same
ST4a   client cancel (Nav2 rejects), Ctrl-C -> "Canceled"
ST4c   client cancel (Nav2 never answers), Ctrl-C -> "Canceled"
X_late49i Nav2 accepts 4.9 s late, rejects the cancel; Ctrl-C 0.3 s after the request -> "Navigator shutting down", exit < 10 s
## Same under ros2 launch (Ctrl-C to the process group): no SIGKILL, client result
G3x_ign, G3x_hang, G3x_dead, G3x_late49i (+ hang-cancel: worst case exit time)
## New behavior
NC1 client cancel, Nav2 rejects it, no Ctrl-C -> goal keeps running (feedback, no result) 6 s; then Ctrl-C -> "Canceled" within ~2.5 s
## Regression
AD1 (Ctrl-C inside the accept callback; 0.1 s after) | ST4b | F1b | X_late45 | N6 (cancel latency) | G3 (launch Ctrl-C mid-drive)
## Carried forward from report 6 (7b9f919, untouched code): AD2a-d, C22b, C22d, F1a, T1, I1, F2a, N7, N8c, N6cli, F1, V14, V15, P1, B1, S0

## Results (HEAD cde8436, 2026-10-26 07:56-08:11; logs in scratchpad qa7/logs; times after Ctrl-C: client result / exit)
PASS S0 sandbox sees only /rosout, /parameter_events
PASS X_ign 2.83 s / 4.94 s | X_hang 2.19 / 3.39 | X_dead 2.25 / 3.38 — all "Navigator shutting down", "Nav2 did not end the canceled goal", no traceback
PASS ST4a "Canceled" 0.58 / 1.68 | ST4c "Canceled" 0.70 / 1.93
PASS X_late49i "Navigator shutting down" 6.72 / 7.92 (late accept 4.4 s after Ctrl-C, canceled 0.27 s after it)
PASS NC1 refused cancel, no Ctrl-C: feedback goes on for 6 s, no result; Ctrl-C -> "Canceled" 0.16 s, exit 1.29 s
PASS launch: G3x_ign 3.37 s, G3x_hang 3.22 s, G3x_dead 3.52 s (no escalation); G3x_late49i 7.91 s, G3x_late4.95+hang 8.23 s (SIGTERM escalation at 5 s, "finished cleanly", no SIGKILL); every client got "Navigator shutting down"
PASS regression: AD1 in-callback 1.17 s, +0.1 s 1.37 s (Nav2 cancel 0.24-0.26 s) | ST4b "Canceled", 2.96 s | F1b 6.47 s (result 5.2 s) | X_late45 5.42 s (result 4.16 s) | N6 cancel latency 0.18 s, result 0.31 s, next goal Reached | G3 1.52 s
CONCERN: X_ign log "exception was never retrieved: Failed to publish feedback ... feedback publisher is invalid" (Nav2 still sends feedback while ROS goes down)
