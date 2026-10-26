# QA report 7 (follow-up) — driving packages

Tested commit: cde8436 (fix since 7b9f919), prior report `qa-report-6.md`. Scenario list: `qa-7-scenarios.md`. Sandbox re-checked (S0 pass); clean rebuild by the coordinator, `install/` matches `src/`.

## Verdict: Passes

Both fails of report 6 are fixed: after Ctrl-C every client gets a result, also when Nav2 rejects, never answers or died before the cancel. Under `ros2 launch` nothing needs SIGKILL. A refused cancel without Ctrl-C keeps the goal running, as designed.

## Reruns of report-6 fails (client result / exit, after Ctrl-C)

| Scenario | Result | Exit |
|---|---|---|
| X_ign (Nav2 rejects the cancel) | "Navigator shutting down" 2.83 s | 4.94 s |
| X_hang (cancel never answered) | same, 2.19 s | 3.39 s |
| X_dead (Nav2 died mid-goal) | same, 2.25 s | 3.38 s |
| ST4a (client cancel rejected, then Ctrl-C) | "Canceled" 0.58 s | 1.68 s |
| ST4c (client cancel unanswered, then Ctrl-C) | "Canceled" 0.70 s | 1.93 s |
| X_late49i (accept 4.9 s late, cancel rejected) | "Navigator shutting down" 6.72 s | 7.92 s |
| launch: G3x_ign / G3x_hang / G3x_dead | result each | 3.37 / 3.22 / 3.52 s, no escalation |
| launch: G3x_late49i / late 4.95 + cancel unanswered | result each | 7.91 / 8.23 s; SIGTERM at 5 s, "finished cleanly", no SIGKILL |

No traceback in any of them; the log names the case ("Nav2 did not end the canceled goal").

## New behavior

NC1: client cancel rejected by Nav2, no Ctrl-C: feedback keeps coming for 6 s and no result (goal keeps running). Ctrl-C then: "Canceled" 0.16 s later, exit 1.29 s.

## Regression — all pass

AD1 (Ctrl-C inside Nav2's accept callback / 0.1 s after) 1.17 / 1.37 s, Nav2 sees the cancel 0.24–0.26 s after Ctrl-C. ST4b "Canceled", 2.96 s. F1b 6.47 s (result 5.2 s). X_late45 5.42 s (result 4.16 s). N6 cancel latency 0.18 s, result 0.31 s, next goal Reached. G3 launch Ctrl-C mid-drive 1.52 s, both nodes finish cleanly.

## Concerns

- After a refused client cancel, Ctrl-C ends the goal at once without re-sending the cancel to Nav2, so a Nav2 that refused once keeps driving.
- X_ign: stray "exception was never retrieved: Failed to publish feedback ... feedback publisher is invalid" at shutdown (Nav2 still sends feedback).
- Late-acceptance worst case exits at ~8.2 s (1.8 s before launch's SIGKILL) and shows launch's SIGTERM escalation message.
- Carried from report 6: client result after Ctrl-C waits for Nav2's pending answer (F1b 5.2 s); a cancel while Nav2 is silent ends after the response timeout.

## Not covered

Real Nav2, real LiDAR (blocked in report 6), systemd, `colcon test`. Carried forward from report 6: AD2a–d, C22b/d, F1a, T1, I1, F2a, N7, N8c, N6cli, F1, V14, V15, P1, B1.
