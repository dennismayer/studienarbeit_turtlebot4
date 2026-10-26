# QA report 5 (follow-up) — driving packages

Tested commit: a33f82d (fix since 79fb9b9), prior report `qa-report-4.md`. Scenario list: `qa-5-scenarios.md`. Isolation re-checked (S0 pass).

## Verdict: Passes

X_late45 is fixed and every shutdown path `stop()` touches still works. Exit times 1.3–6.1 s in the required cases, 7.8 s in the worst case built, all under ros2 launch's 10 s SIGKILL.

## Reruns — all pass

| Scenario | Exit | Result |
|---|---|---|
| X_late45 (Nav2 accepts 4.5 s after the request, Ctrl-C 0.5 s after it got it) | 5.32 s | client "Navigator shutting down" after 1 s; acceptance canceled 0.14 s after it arrived |
| F1a (Nav2 absent) | 1.25 s | "Navigator shutting down", attempts 0 |
| F1b (Nav2 silent, timeout 8 s) | 6.12 s | client result after 1 s; node waits the 5 s cap |
| C22d (accept 1 s late) | 1.85 s | acceptance canceled, result delivered |
| T1 (SIGTERM during a drive) | 1.27 s | canceled, result delivered |
| I1 (started with SIGINT ignored) | 1.28 s | canceled, result delivered |
| F2a (next goal during shutdown) | 1.50 s | canceled, result delivered, next goal rejected, no second Nav2 goal |

Exploratory, pass: accept 4.9 s late and reject the cancel (5.75 s exit); accept 4.9 s late and never answer the cancel (7.80 s, worst case); `ros2 launch` with an acceptance 4.5 s late (canceled, both nodes finish cleanly, no escalation). Carried forward from report 4: C22b, F2b, G3, D1, double Ctrl-C, X_late3.

## Concern

**X_late55** — an acceptance arriving after the response deadline (5.5 s after the request, default 5 s timeout; Ctrl-C 0.3 s after the request) is still left running: the Navigator exits after 5.69 s, the acceptance arrives later and is never canceled. Matches the spec ("until nav2_response_timeout would have run out"); the documented limit should name this case too. Log also shows a stray "exception was never retrieved: cannot use Destroyable because destruction was requested".

## Not covered

Real Nav2, a real person in front of the LiDAR (user step), systemd-managed shutdown, `colcon test`.
