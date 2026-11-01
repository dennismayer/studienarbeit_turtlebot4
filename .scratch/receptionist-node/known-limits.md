# Receptionist node: known limits

Accepted for now; revisit when they start to matter.

## Failed drive home leaves the node in an unclear state

Where: `ReceptionistNode._execute_callback`, the `finally` block that drives back to the station.

When the drive home fails, the callback raises `RuntimeError` inside `finally`. Intended behaviour is that the robot stops serving visitors. What actually happens:

- rclpy catches the exception (`rclpy/action/server.py`, `_execute_goal`), so the process keeps running and `main` never sees it.
- `self._lock_callback.release()` is skipped, so every later visitor goal is rejected, logged as "another request is still running" instead of the real reason.
- On a normal finish, `goal_handle.succeed()` already ran, so the failed visit is reported as SUCCEEDED with an empty result.

Possible fix when needed: either an explicit out-of-service flag checked in `_goal_callback` (process stays up), or shut down the ROS context so `main` exits. In both cases the visit should end with `abort()`.

## Unknown professor writes a placeholder professor to memory

Where: `ReceptionistNode._react_to_standard_message`, extraction runs before the transition is applied.

For `unknown_professor`, the model still has to pick a professor id (the field is required), and that id is saved to memory. Harmless: the next `professor_given` answer overwrites it before anything reads it.
