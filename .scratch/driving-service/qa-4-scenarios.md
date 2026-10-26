# Follow-up QA 4: prior qa-report-3 at 98072da, now 79fb9b9
Fix diff: navigator_node (_stopping checks, reject during shutdown), run_node (flag-only SIGINT/SIGTERM handler).
## Never and always
- Never: Nav2 driving with nobody owning the goal; a new Nav2 goal after a stop signal; a client left without a result.
- Always: SIGINT and SIGTERM end both nodes cleanly, also when SIGINT was ignored at start.
## Reruns of report-3 fails
F1a Ctrl-C while waiting for absent Nav2 -> aborted "Navigator shutting down"
F1b Ctrl-C with silent Nav2, nav2_response_timeout:=8.0 -> same
F2a C22e: Ctrl-C mid-drive, main-loop stand-in sends next goal at once -> next goal rejected, no new Nav2 goal
F2b F1 flow under ros2 launch with Ctrl-C mid-drive
## Report-3 concerns, now requirements
T1  SIGTERM mid-drive -> Nav2 goal canceled, client "Navigator shutting down"
I1  Navigator started with SIGINT ignored (`&`) -> Ctrl-C mid-drive works
## Affected reruns
C22a (=N13) Ctrl-C mid-drive; C22b Ctrl-C in retry delay; C22d Ctrl-C with late (1 s) acceptance; G3 launch Ctrl-C
D1  VisitorDetector: Ctrl-C idle, Ctrl-C during an open visit, SIGTERM, SIGINT-ignored start
## Exploratory: shutdown edges (late acceptance after stop window, double Ctrl-C, signal during startup, launch SIGTERM escalation)
## Carried forward from report 3 (98072da): X2, X2v, N11, N15, V9a/b/c, F1 visit flow; earlier reports for the rest

## Results (HEAD 79fb9b9, 2026-10-25 21:43-21:57)
PASS: S0; F1a (F1a_summary.log); F1b (F1b_summary.log); F2a (F2a_*); F2b + G3 SIGINT (F2b_*); T1 SIGTERM (T1_summary.log); I1 ignored SIGINT (I1_summary.log)
PASS: C22a (=I1/T1 drive cases), C22b (C22b_summary.log), C22d (C22d_summary.log); D1 detector SIGINT/SIGTERM idle + open visit + ignored-SIGINT start (D1_*, sigign_probe.log)
PASS: exploratory double Ctrl-C (X_double_*), late acceptance 2.5 s after Ctrl-C canceled (X_late3_*)
FAIL (edge): X_late45 Nav2 accepts a pre-Ctrl-C request 4.5 s late (after the Navigator exited) -> Nav2 drives 30 s unowned (X_late45b_*)
CONCERN: SIGTERM to the ros2 launch process kills launch only; both nodes keep running orphaned (G3t_*); SIGINT during module import prints a KeyboardInterrupt traceback (X_startup_nav.log)
