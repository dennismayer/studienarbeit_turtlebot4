# The visitor detector is the action client, the receptionist main loop is the server

The LiDAR visitor detector in `driving_service` sends a `VisitorDetected` goal to the receptionist main loop, which serves it and returns the result only when the whole **Visit** is over, including any guiding drive and the drive back. This makes a detected **Visitor** the event that starts the application, and the open goal is what pauses detection during a Visit, so neither side needs a second pause or resume interface.

## Considered Options

- **Main loop asks a detector server ("wait for a visitor") and gets the result when one appears.** That's the more usual direction, and it leaves the main loop in charge of when detection runs. We didn't take it because the detector would then need its own idle/armed state driven by the caller, and the start of the application would be a request instead of an event.
- **Detector publishes a topic, as the old guide code did.** We didn't take it because a topic has no acceptance, no "busy" answer and no end, so the detector can't tell whether a Visit is still going on.

## Consequences

- If the receptionist server isn't running, the detector keeps detecting and logs that nobody is serving the goal. It doesn't queue detections.
- Swapping the direction later means changing the action definition and both nodes.
