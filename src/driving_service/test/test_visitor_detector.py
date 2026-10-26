"""Behavior of the LiDAR visitor detector, fed with synthetic scans."""

import math
import warnings

from driving_service.visitor_detector import DetectionParams, VisitorDetector
import pytest

# Scan geometry of the TurtleBot4 RPLIDAR: 720 points over a full turn,
# straight ahead of the robot is scan angle -pi/2.
ANGLE_MIN = -math.pi + 0.0175
ANGLE_INCREMENT = 2 * math.pi / 720
NUM_POINTS = 720
RANGE_MIN = 0.15
RANGE_MAX = 12.0
FRONT = -math.pi / 2


def make_params(**overrides):
    values = {
        'min_distance': 0.8,
        'max_distance': 2.5,
        'angle_range': 60.0,
        'person_width_min': 0.35,
        'person_width_max': 0.7,
        'stability_threshold': 8,
        'visitor_timeout': 10.0,
        'cooldown_period': 5.0,
        'min_confidence': 0.75,
        'frame_skip_rate': 1,
        'lidar_front_angle': FRONT,
    }
    values.update(overrides)
    return DetectionParams(**values)


def empty_ranges():
    return [math.inf] * NUM_POINTS


def add_object(ranges, distance, angle, width, front=FRONT):
    """Put a flat object of the given width at distance/angle from the robot heading."""
    half_span = math.atan2(width / 2, distance)
    center = front + angle
    for i in range(NUM_POINTS):
        scan_angle = ANGLE_MIN + i * ANGLE_INCREMENT
        offset = math.atan2(math.sin(scan_angle - center), math.cos(scan_angle - center))
        if abs(offset) <= half_span:
            ranges[i] = distance / math.cos(offset)
    return ranges


def add_points(ranges, distance, count):
    """Put count neighboring readings at distance, straight ahead."""
    first = round((FRONT - ANGLE_MIN) / ANGLE_INCREMENT)
    for i in range(first, first + count):
        ranges[i] = distance
    return ranges


def scan_with_person(distance=1.2, angle=0.0, width=0.5):
    return add_object(empty_ranges(), distance, angle, width)


class Clock:
    """Scan timestamps at the RPLIDAR rate."""

    def __init__(self):
        self.now = 1000.0

    def tick(self, seconds=0.135):
        self.now += seconds
        return self.now


def feed(detector, clock, ranges, scans):
    """Feed the same scan several times, return every visitor that was reported."""
    reported = []
    for _ in range(scans):
        visitor = detector.process_scan(
            ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX, clock.tick())
        if visitor is not None:
            reported.append(visitor)
    return reported


def test_person_standing_in_front_is_reported_with_distance_and_angle():
    detector = VisitorDetector(make_params())
    clock = Clock()

    reported = feed(detector, clock, scan_with_person(distance=1.2, angle=0.1), 10)

    assert len(reported) == 1
    assert reported[0].distance == pytest.approx(1.2, abs=0.05)
    assert reported[0].angle == pytest.approx(0.1, abs=0.03)
    assert 0.75 <= reported[0].confidence <= 1.0


def test_object_wider_than_a_person_is_not_reported_even_without_confidence_threshold():
    detector = VisitorDetector(make_params(min_confidence=0.0))
    clock = Clock()

    reported = feed(detector, clock, scan_with_person(distance=1.2, width=0.8), 20)

    assert reported == []


def test_person_further_away_than_detection_range_is_not_reported_even_at_low_threshold():
    detector = VisitorDetector(make_params(min_confidence=0.4))
    clock = Clock()

    reported = feed(detector, clock, scan_with_person(distance=3.0), 20)

    assert reported == []


def test_person_closer_than_detection_range_is_not_reported_even_at_low_threshold():
    detector = VisitorDetector(make_params(min_confidence=0.4))
    clock = Clock()

    reported = feed(detector, clock, scan_with_person(distance=0.5), 20)

    assert reported == []


def test_person_outside_the_front_window_is_not_reported():
    detector = VisitorDetector(make_params())
    clock = Clock()

    reported = feed(detector, clock, scan_with_person(angle=math.radians(45)), 20)

    assert reported == []


def test_person_seen_in_only_every_other_scan_is_not_reported():
    detector = VisitorDetector(make_params())
    clock = Clock()

    reported = []
    for _ in range(10):
        reported += feed(detector, clock, scan_with_person(), 1)
        reported += feed(detector, clock, empty_ranges(), 1)

    assert reported == []


def test_person_who_stays_is_reported_only_once_during_a_visit():
    detector = VisitorDetector(make_params())
    clock = Clock()

    reported = feed(detector, clock, scan_with_person(), 500)

    assert len(reported) == 1


def test_next_visitor_is_reported_after_the_scene_was_clear_and_the_cooldown_passed():
    detector = VisitorDetector(make_params(visitor_timeout=10.0, cooldown_period=5.0))
    clock = Clock()
    feed(detector, clock, scan_with_person(), 10)

    detector.visit_finished(clock.now)
    reported_too_early = feed(detector, clock, empty_ranges(), 100)  # 13.5 s
    clock.tick(2.0)  # 15.5 s
    reported = feed(detector, clock, scan_with_person(distance=1.5), 10)

    assert reported_too_early == []
    assert len(reported) == 1
    assert reported[0].distance == pytest.approx(1.5, abs=0.05)


def test_visitor_who_stays_after_the_visit_is_not_greeted_again():
    detector = VisitorDetector(make_params(visitor_timeout=10.0, cooldown_period=5.0))
    clock = Clock()
    feed(detector, clock, scan_with_person(), 10)

    detector.visit_finished(clock.now)
    reported = feed(detector, clock, scan_with_person(), 450)  # about 60 s

    assert reported == []


def test_visitor_who_walks_up_during_the_cooldown_is_greeted_after_it():
    detector = VisitorDetector(make_params(visitor_timeout=10.0, cooldown_period=5.0))
    clock = Clock()
    feed(detector, clock, scan_with_person(), 10)

    detector.visit_finished(clock.now)
    feed(detector, clock, empty_ranges(), 82)  # about 11 s: clear, cooldown running
    reported_in_cooldown = feed(detector, clock, scan_with_person(), 22)  # until about 14 s
    reported = feed(detector, clock, scan_with_person(), 40)  # waits about 5 s more

    assert reported_in_cooldown == []
    assert len(reported) == 1


def test_visitor_is_reported_again_after_a_visit_that_never_started():
    detector = VisitorDetector(make_params())
    clock = Clock()
    feed(detector, clock, scan_with_person(), 10)

    detector.visit_not_started()
    reported_before_stable_again = feed(detector, clock, scan_with_person(), 7)
    reported = feed(detector, clock, scan_with_person(), 1)

    assert reported_before_stable_again == []
    assert len(reported) == 1


def test_frame_skip_rate_processes_only_every_nth_scan():
    detector = VisitorDetector(make_params(stability_threshold=8, frame_skip_rate=3))
    clock = Clock()

    reported_early = feed(detector, clock, scan_with_person(), 23)
    reported = feed(detector, clock, scan_with_person(), 1)

    assert reported_early == []
    assert len(reported) == 1


def test_visitor_who_steps_aside_briefly_after_the_visit_is_not_greeted_again():
    detector = VisitorDetector(make_params(visitor_timeout=10.0, cooldown_period=5.0))
    clock = Clock()
    feed(detector, clock, scan_with_person(), 10)

    detector.visit_finished(clock.now)
    reported = []
    for _ in range(4):
        reported += feed(detector, clock, scan_with_person(), 30)  # about 4 s in view
        reported += feed(detector, clock, empty_ranges(), 22)  # about 3 s out of view

    assert reported == []


def test_single_glimpse_after_the_cooldown_is_not_reported():
    detector = VisitorDetector(make_params(visitor_timeout=10.0, cooldown_period=5.0))
    clock = Clock()
    feed(detector, clock, scan_with_person(), 10)

    detector.visit_finished(clock.now)
    feed(detector, clock, empty_ranges(), 82)  # about 11 s: clear, cooldown running
    clock.tick(5.0)  # the next scan re-arms the detector
    reported = feed(detector, clock, scan_with_person(), 1)

    assert reported == []


def test_visitor_is_reported_even_when_the_deciding_scan_misses_them():
    detector = VisitorDetector(make_params(stability_threshold=8))
    clock = Clock()

    feed(detector, clock, scan_with_person(distance=1.3), 7)
    reported = feed(detector, clock, empty_ranges(), 1)

    assert len(reported) == 1
    assert reported[0].distance == pytest.approx(1.3, abs=0.05)


def test_the_more_person_like_of_two_clusters_is_reported():
    detector = VisitorDetector(make_params(min_confidence=0.5))
    clock = Clock()
    ranges = add_object(empty_ranges(), distance=1.0, angle=-0.3, width=0.52)
    ranges = add_object(ranges, distance=2.0, angle=0.3, width=0.45)

    reported = feed(detector, clock, ranges, 10)

    assert len(reported) == 1
    assert reported[0].distance == pytest.approx(1.0, abs=0.05)


def test_barely_person_wide_object_at_the_far_edge_is_not_confident_enough():
    detector = VisitorDetector(make_params(min_confidence=0.75))
    clock = Clock()

    reported = feed(detector, clock, scan_with_person(distance=2.4, width=0.40), 20)

    assert reported == []


def test_confidence_falls_from_about_1_near_to_about_085_at_the_far_edge():
    clock = Clock()
    near_person = scan_with_person(distance=0.85, width=0.53)
    far_person = scan_with_person(distance=2.45, width=0.53)
    near = feed(VisitorDetector(make_params()), clock, near_person, 10)
    far = feed(VisitorDetector(make_params()), clock, far_person, 10)

    assert [v.confidence for v in near] == pytest.approx([0.99], abs=0.03)
    assert [v.confidence for v in far] == pytest.approx([0.855], abs=0.03)


def scan_with_legs_apart(distance=1.2, background=None):
    """Two 0.12 m legs 0.15 m apart, as the LiDAR sees them at ankle height."""
    ranges = empty_ranges()
    if background is not None:
        ranges = add_object(ranges, distance=background, angle=0.0, width=6.0)
    for side in (-1, 1):
        add_object(ranges, distance, angle=side * math.atan2(0.135, distance), width=0.12)
    return ranges


@pytest.mark.parametrize('background', [None, 4.0])
def test_person_with_legs_apart_is_reported_as_one_visitor(background):
    detector = VisitorDetector(make_params(person_width_min=0.15, person_width_max=0.55))
    clock = Clock()

    reported = feed(detector, clock, scan_with_legs_apart(background=background), 10)

    assert len(reported) == 1
    assert reported[0].distance == pytest.approx(1.2, abs=0.05)
    assert reported[0].angle == pytest.approx(0.0, abs=0.03)


def test_person_standing_in_front_of_a_wall_is_reported():
    detector = VisitorDetector(make_params())
    clock = Clock()
    ranges = add_object(empty_ranges(), distance=2.0, angle=0.0, width=3.0)
    ranges = add_object(ranges, distance=1.2, angle=0.0, width=0.5)

    reported = feed(detector, clock, ranges, 10)

    assert len(reported) == 1
    assert reported[0].distance == pytest.approx(1.2, abs=0.05)


def test_readings_below_the_sensor_minimum_are_ignored():
    detector = VisitorDetector(make_params())
    clock = Clock()
    ranges = scan_with_person(distance=1.2)
    person = [i for i, r in enumerate(ranges) if math.isfinite(r)]
    for i in person[1::4]:
        ranges[i] = 0.0  # no return, reported as 0 by some drivers

    reported = feed(detector, clock, ranges, 10)

    assert len(reported) == 1
    assert reported[0].distance == pytest.approx(1.2, abs=0.05)


def test_readings_above_the_sensor_maximum_are_ignored():
    detector = VisitorDetector(make_params())
    clock = Clock()
    ranges = scan_with_person(distance=1.2)
    person = [i for i, r in enumerate(ranges) if math.isfinite(r)]
    for i in person[1::4]:
        ranges[i] = RANGE_MAX + 1.0  # no return, reported above range_max by some drivers

    reported = feed(detector, clock, ranges, 10)

    assert len(reported) == 1
    assert reported[0].distance == pytest.approx(1.2, abs=0.05)


@pytest.mark.parametrize('points, reports', [(2, 0), (3, 1)])
def test_cluster_needs_three_points_even_when_its_width_fits(points, reports):
    detector = VisitorDetector(make_params(
        person_width_min=0.001, person_width_max=0.05, min_confidence=0.0))
    clock = Clock()

    reported = feed(detector, clock, add_points(empty_ranges(), 1.2, points), 10)

    assert len(reported) == reports


def test_object_narrower_than_a_person_is_not_reported_even_without_confidence_threshold():
    detector = VisitorDetector(make_params(min_confidence=0.0))
    clock = Clock()

    reported = feed(detector, clock, scan_with_person(distance=1.2, width=0.25), 20)

    assert reported == []


def test_person_ahead_is_found_when_the_front_lies_at_the_scan_seam():
    detector = VisitorDetector(make_params(lidar_front_angle=math.pi))
    clock = Clock()
    ranges = add_object(empty_ranges(), distance=1.2, angle=0.0, width=0.5, front=math.pi)

    reported = feed(detector, clock, ranges, 10)

    assert len(reported) == 1
    assert reported[0].angle == pytest.approx(0.0, abs=0.03)


def test_empty_front_window_is_processed_without_warnings():
    detector = VisitorDetector(make_params())
    clock = Clock()

    with warnings.catch_warnings():
        warnings.simplefilter('error')
        reported = feed(detector, clock, empty_ranges(), 3)

    assert reported == []


@pytest.mark.parametrize('bad', [
    {'stability_threshold': 2},
    {'frame_skip_rate': 0},
    {'cooldown_period': -1.0},
    {'visitor_timeout': -1.0},
    {'angle_range': 400.0},
    {'angle_range': 0.0},
    {'min_confidence': -0.1},
    {'min_confidence': 1.5},
    {'person_width_min': 0.7, 'person_width_max': 0.35},
    {'min_distance': 2.5, 'max_distance': 2.5},
])
def test_invalid_detection_params_are_rejected(bad):
    with pytest.raises(ValueError):
        make_params(**bad)
