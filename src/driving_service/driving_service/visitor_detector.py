"""
LiDAR visitor detection, without ROS.

Created with Claude (Anthropic) on the basis of the car_museum_guide package.
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import List, Optional, Sequence

from driving_service.params import check
import numpy as np

# neighboring points further apart than this start a new cluster
MAX_POINT_GAP = 0.1

WIDTH_SCORE_WEIGHT = 0.5
DISTANCE_SCORE_WEIGHT = 0.3
CLUSTER_SIZE_SCORE_WEIGHT = 0.2
# the far end of the distance range still scores 1 - this
DISTANCE_SCORE_FALLOFF = 0.5
# clusters with at least this many points get the full size score
FULL_SIZE_SCORE_POINTS = 10.0


@dataclass(frozen=True)
class DetectionParams:
    """Tuning of the detector, see config/ros_params.yaml for the meaning of each value."""

    min_distance: float
    max_distance: float
    angle_range: float
    person_width_min: float
    person_width_max: float
    stability_threshold: int
    visitor_timeout: float
    cooldown_period: float
    min_confidence: float
    frame_skip_rate: int
    lidar_front_angle: float

    def __post_init__(self):
        """Reject values the detector cannot work with."""
        check([
            (self.min_distance < self.max_distance,
             'detection_distance_min must be < detection_distance_max'),
            (0.0 < self.angle_range <= 360.0, 'detection_angle_range must be within (0, 360]'),
            (self.person_width_min < self.person_width_max,
             'person_width_min must be < person_width_max'),
            (self.stability_threshold >= 3, 'stability_threshold must be >= 3'),
            (self.visitor_timeout >= 0.0, 'visitor_timeout must be >= 0'),
            (self.cooldown_period >= 0.0, 'cooldown_period must be >= 0'),
            (0.0 <= self.min_confidence <= 1.0, 'min_confidence must be within 0..1'),
            (self.frame_skip_rate >= 1, 'lidar_frame_skip_rate must be >= 1'),
        ])


@dataclass(frozen=True)
class VisitorCandidate:
    """A person-like cluster in front of the robot; reported once it is stable."""

    distance: float
    angle: float
    confidence: float


class _State(Enum):
    ARMED = 0               # looking for a stable detection
    IN_VISIT = 1            # reported, waiting for visit_finished()
    WAITING_FOR_CLEAR = 2   # no candidate may be seen for visitor_timeout
    COOLDOWN = 3            # scans are ignored until the cooldown is over


class VisitorDetector:
    """Turns a stream of scans into a single report per visit."""

    def __init__(self, params: DetectionParams):
        self._params = params
        self._history: deque = deque(maxlen=params.stability_threshold)
        self._latest: Optional[VisitorCandidate] = None
        self._state = _State.ARMED
        self._scan_count = 0
        self._last_seen = 0.0
        self._cooldown_until = 0.0

    def process_scan(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        now: float,
    ) -> Optional[VisitorCandidate]:
        """Process one scan; return the visitor to report, or None."""
        self._scan_count += 1
        if self._scan_count % self._params.frame_skip_rate != 0:
            return None

        candidate = self._best_candidate(ranges, angle_min, angle_increment, range_min)
        if self._state is _State.IN_VISIT:
            return None
        if self._state is _State.WAITING_FOR_CLEAR:
            if candidate is not None:
                self._last_seen = now
            elif now - self._last_seen >= self._params.visitor_timeout:
                self._state = _State.COOLDOWN
                self._cooldown_until = now + self._params.cooldown_period
            return None
        if self._state is _State.COOLDOWN:
            if now < self._cooldown_until:
                return None
            self._state = _State.ARMED

        self._history.append(candidate is not None)
        if candidate is not None:
            self._latest = candidate

        if self._is_stable():
            self._state = _State.IN_VISIT
            return self._latest
        return None

    def visit_finished(self, now: float) -> None:
        """Re-arm once the scene is clear for visitor_timeout and the cooldown is over."""
        self._state = _State.WAITING_FOR_CLEAR
        self._last_seen = now
        self._history.clear()

    def visit_not_started(self) -> None:
        """Arm again right away, e.g. when nobody served the reported visitor."""
        self._state = _State.ARMED
        self._history.clear()

    def _is_stable(self) -> bool:
        threshold = self._params.stability_threshold
        return len(self._history) == threshold and sum(self._history) >= threshold - 2

    def _best_candidate(self, ranges, angle_min, angle_increment, range_min):
        ranges = np.asarray(ranges, dtype=float)
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        # angle relative to the robot heading, wrapped to [-pi, pi]
        relative = np.arctan2(
            np.sin(angles - self._params.lidar_front_angle),
            np.cos(angles - self._params.lidar_front_angle))

        half_window = math.radians(self._params.angle_range / 2.0)
        mask = (
            np.isfinite(ranges)
            & (ranges >= range_min)
            & (np.abs(relative) <= half_window)
        )
        order = np.argsort(relative[mask])
        ranges, relative = ranges[mask][order], relative[mask][order]

        best = None
        for cluster_ranges, cluster_angles in _clusters(ranges, relative):
            candidate = self._score(cluster_ranges, cluster_angles)
            if candidate is not None and (best is None or candidate.confidence > best.confidence):
                best = candidate
        return best

    def _score(self, ranges: np.ndarray, angles: np.ndarray) -> Optional[VisitorCandidate]:
        p = self._params
        distance = float(np.mean(ranges))
        if not p.min_distance <= distance <= p.max_distance:
            return None
        x, y = ranges * np.cos(angles), ranges * np.sin(angles)
        width = float(math.hypot(x[-1] - x[0], y[-1] - y[0]))
        if not p.person_width_min <= width <= p.person_width_max:
            return None

        optimal_width = (p.person_width_min + p.person_width_max) / 2.0
        max_deviation = (p.person_width_max - p.person_width_min) / 2.0
        width_score = 1.0 - abs(width - optimal_width) / max_deviation
        normalized = (distance - p.min_distance) / (p.max_distance - p.min_distance)
        distance_score = 1.0 - normalized * DISTANCE_SCORE_FALLOFF
        size_score = min(len(ranges) / FULL_SIZE_SCORE_POINTS, 1.0)

        confidence = (
            width_score * WIDTH_SCORE_WEIGHT
            + distance_score * DISTANCE_SCORE_WEIGHT
            + size_score * CLUSTER_SIZE_SCORE_WEIGHT
        )
        if confidence < p.min_confidence:
            return None
        return VisitorCandidate(
            distance=distance, angle=float(np.mean(angles)), confidence=confidence)


def _clusters(ranges: np.ndarray, angles: np.ndarray) -> List:
    """Split points ordered by angle into runs of neighbors."""
    if len(ranges) == 0:
        return []
    x, y = ranges * np.cos(angles), ranges * np.sin(angles)
    gaps = np.hypot(np.diff(x), np.diff(y))
    splits = np.flatnonzero(gaps > MAX_POINT_GAP) + 1
    return list(zip(np.split(ranges, splits), np.split(angles, splits)))
