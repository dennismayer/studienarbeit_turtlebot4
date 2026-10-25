# Spec: driving_interfaces + driving_service

Status: in progress
Branch: feat/driving-service (base: main @ 371feb3)

Settled in /grill-with-docs on 2026-10-25. Terms: see `GLOSSARY.md` (Visitor, Visit, Location). Decision record: `docs/adr/0001-visitor-detector-is-the-action-client.md`.

These two packages are agent-owned (see the exception in `AGENTS.md`).

## Requirements

### General
1. Two new packages: `src/driving_interfaces` (ament_cmake, interface definitions only) and `src/driving_service` (ament_python), laid out like `speaking_interfaces` / `speaking_service`.
2. Every package carries a label that it was created with Claude on the basis of the car_museum_guide package. That's the only place that package is mentioned; the code is about the receptionist bot only.
3. ROS 2 Jazzy, system Python. Parameters in `config/ros_params.yaml`, both nodes started by `launch/driving_service.launch.py`.

### driving_interfaces
4. `action/MoveToLocation.action`:
   - Goal: `float64 x`, `float64 y` (map frame, m), `float64 yaw` (rad, 0 = map x axis, CCW positive).
   - Result: `bool success`, `string message` (`"Reached"`, `"Canceled"`, `"Nav2Unavailable"` or the failure description), `uint8 attempts`.
   - Feedback: `uint8 attempt` (starts at 1), `float32 distance_remaining` (m, from Nav2).
5. `action/VisitorDetected.action`:
   - Goal: `float32 distance` (m, LiDAR to visitor cluster), `float32 angle` (rad, relative to the robot heading, 0 = ahead, left positive), `float32 confidence` (0..1).
   - Result: `bool success`, `string message`.
   - No feedback.

### Navigator node (driving_service) — MoveToLocation server
6. Serves `MoveToLocation` by sending `nav2_msgs/action/NavigateToPose` goals to Nav2 directly (no BasicNavigator).
7. One goal at a time: a goal that arrives while one is running is rejected.
8. When Nav2 fails a goal, it's retried up to `nav_retry_attempts` (default 3) times total, with `nav_retry_delay` (default 5 s) between attempts. Feedback carries the current attempt and Nav2's distance_remaining.
9. Canceling the MoveToLocation goal cancels the running Nav2 goal (and any pending retry); the result is `"Canceled"`.
10. If the Nav2 action server isn't available, the goal fails with `"Nav2Unavailable"` (no retries).
11. The node does not set the initial pose. Localization is done outside (RViz 2D Pose Estimate or AMCL params); goals fail with a clear message until then.

### Visitor detector node (driving_service) — VisitorDetected client
12. Continuously clusters `/scan` (best-effort sensor QoS): valid ranges only, front window ±`detection_angle_range`/2 around `lidar_front_angle` (the scan angle that points straight ahead; default −π/2 because the TurtleBot4 URDF mounts the LiDAR rotated +90°), clusters by gap between neighboring points, min points per cluster.
13. A cluster is a visitor candidate when its mean distance is within [`detection_distance_min`, `detection_distance_max`], its width is within [`person_width_min`, `person_width_max`], and its confidence (weighted width/distance/cluster-size score) is ≥ `min_confidence`.
14. Stable detection: at least `stability_threshold - 2` of the last `stability_threshold` processed scans had a candidate. `lidar_frame_skip_rate` thins processing.
15. On a stable detection while armed, it sends one `VisitorDetected` goal with the best candidate's distance, angle and confidence, and waits for the result. No new goal while one is open; the detector never cancels it.
16. Re-arming after a Visit (result received, goal rejected, or server error): only once no candidate has been seen for `visitor_timeout` seconds, and then after `cooldown_period` more seconds. The same re-arm applies if the goal is rejected.
17. If no VisitorDetected server is available, the detector logs it (throttled) and stays armed; it does not queue detections.

### Out of scope
- The VisitorDetected server (the receptionist main loop in `receptionist_bot`) — the user writes it.
- Return-to-reception logic and the list of saved Locations — owned by the receptionist.
- Moving the map out of the old package.

## Progress

### Done
- Grilling, glossary, ADR 0001.
- Implemented test-first and mutation-checked (commits af7a08c..71f960b): `driving_interfaces`; `driving_service` with `visitor_detector.py` (pure logic), `navigator_node.py`, `visitor_detector_node.py`, `run_node.py`, `config/ros_params.yaml`, `launch/driving_service.launch.py`.
- Mutation survivor (equivalent, kept): the explicit "rejected goal" branch in `VisitorDetectorNode._on_goal_response`; rclpy also answers a result request for a rejected goal, so the outcome is the same, but relying on that is fragile.

### Next steps
- QA, then code review.

### How to run
- Gate: `colcon build --packages-select driving_interfaces driving_service && colcon test --packages-select driving_service && colcon test-result --test-result-base build/driving_service`
- ROS tests run in ROS domain 87 with the discovery server disabled (`ROS_DISCOVERY_SERVER` unset, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) and unique names, so they never reach the robot's real Nav2. The domain ID alone does not isolate on this robot: its Fast DDS discovery server (`127.0.0.1:11811`) connects participants across domains (verified 2026-10-25). Do the same for any manual experiment: `env -u ROS_DISCOVERY_SERVER ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ...`.
- Launch: `ros2 launch driving_service driving_service.launch.py`
