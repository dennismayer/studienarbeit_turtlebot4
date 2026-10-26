# Follow-up QA 5: prior qa-report-4 at 79fb9b9, now a33f82d
Fix diff: navigator_node.stop() waits for a pending Nav2 answer up to the response-timeout deadline (max 5 s).
## Never and always
- Never: a Nav2 acceptance within the response timeout left driving after Ctrl-C; a client without result.
- Always: Navigator exits well under ros2 launch's 10 s SIGKILL.
## Reruns
X_late45  accept 4.5 s after request, Ctrl-C 0.5 s after Nav2 got it -> canceled, client "Navigator shutting down"
F1a F1b C22d T1 I1 F2a  (stop() paths) + exit time of each
## Exploratory: late:4.9, late:4.9 + Nav2 ignores cancel (worst-case exit time), late:5.5 (after the deadline), launch Ctrl-C with a late acceptance
## Carried forward from report 4 (79fb9b9): C22b, F2b, G3, D1, double Ctrl-C, X_late3, and earlier

## Results (HEAD a33f82d, 2026-10-25 22:18-22:28)
PASS X_late45: acceptance 4.5 s after the request canceled, client "Navigator shutting down", exit 5.32 s
PASS F1a 1.25 s | F1b (silent, timeout 8) 6.12 s | C22d 1.85 s | T1 SIGTERM 1.27 s | I1 ignored SIGINT 1.28 s | F2a 1.50 s, next goal rejected
PASS exploratory: late 4.9 s + cancel rejected -> 5.75 s; late 4.9 s + cancel never answered -> 7.80 s (worst seen); launch Ctrl-C + late 4.5 s -> canceled, "finished cleanly", no escalation
CONCERN: X_late55 acceptance after the response deadline (5.5 s, default timeout) during Ctrl-C is orphaned + "exception never retrieved: cannot use Destroyable"
