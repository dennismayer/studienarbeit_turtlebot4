#!/usr/bin/env python3
"""
Visitor Detection Node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
from collections import deque
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass

# Constants
DEFAULT_DETECTION_DISTANCE_MIN = 0.8
DEFAULT_DETECTION_DISTANCE_MAX = 2.5
DEFAULT_DETECTION_ANGLE_RANGE = 60.0
DEFAULT_PERSON_WIDTH_MIN = 0.35
DEFAULT_PERSON_WIDTH_MAX = 0.7
DEFAULT_STABILITY_THRESHOLD = 8
DEFAULT_VISITOR_TIMEOUT = 10.0
DEFAULT_COOLDOWN_PERIOD = 5.0
DEFAULT_MIN_CONFIDENCE = 0.75
DEFAULT_FRAME_SKIP_RATE = 3
MIN_CLUSTER_POINTS = 5
DEFAULT_MAX_GAP_METERS = 0.1
MIN_VALID_WIDTH = 0.0
OPTIMAL_DISTANCE_WEIGHT = 0.5
WIDTH_SCORE_WEIGHT = 0.5
DISTANCE_SCORE_WEIGHT = 0.3
CLUSTER_SIZE_SCORE_WEIGHT = 0.2
MAX_CLUSTER_SIZE_NORMALIZATION = 10.0


@dataclass
class DetectionParams:
    """Container for detection parameters with validation"""
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
    
    def __post_init__(self):
        """Validate parameters"""
        if self.min_distance >= self.max_distance:
            raise ValueError("min_distance must be < max_distance")
        
        if self.person_width_min >= self.person_width_max:
            raise ValueError("person_width_min must be < person_width_max")
        
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        
        if self.stability_threshold < 1:
            raise ValueError("stability_threshold must be >= 1")


class ClusterAnalyzer:
    """Analyzes LiDAR point clusters"""
    
    @staticmethod
    def find_clusters(
        ranges: np.ndarray,
        angles: np.ndarray,
        max_gap: float = DEFAULT_MAX_GAP_METERS
    ) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Group nearby LiDAR points into clusters.
        
        Args:
            ranges: Array of distance measurements
            angles: Array of corresponding angles
            max_gap: Maximum euclidean distance to consider same cluster
            
        Returns:
            List of (ranges, angles) tuples for each cluster
        """
        if len(ranges) == 0:
            return []
        
        clusters: List[Tuple[np.ndarray, np.ndarray]] = []
        current_ranges = [ranges[0]]
        current_angles = [angles[0]]
        
        for i in range(1, len(ranges)):
            distance = ClusterAnalyzer._calculate_point_distance(
                ranges[i-1], angles[i-1],
                ranges[i], angles[i]
            )
            
            if distance <= max_gap:
                current_ranges.append(ranges[i])
                current_angles.append(angles[i])
            else:
                if len(current_ranges) >= MIN_CLUSTER_POINTS:
                    clusters.append((
                        np.array(current_ranges),
                        np.array(current_angles)
                    ))
                
                current_ranges = [ranges[i]]
                current_angles = [angles[i]]
        
        if len(current_ranges) >= MIN_CLUSTER_POINTS:
            clusters.append((
                np.array(current_ranges),
                np.array(current_angles)
            ))
        
        return clusters
    
    @staticmethod
    def _calculate_point_distance(
        r1: float, theta1: float,
        r2: float, theta2: float
    ) -> float:
        """Calculate euclidean distance between two polar points"""
        x1, y1 = r1 * np.cos(theta1), r1 * np.sin(theta1)
        x2, y2 = r2 * np.cos(theta2), r2 * np.sin(theta2)
        return float(np.sqrt((x2 - x1)**2 + (y2 - y1)**2))
    
    @staticmethod
    def estimate_width(ranges: np.ndarray, angles: np.ndarray) -> float:
        """
        Estimate the width of an object from scan points.
        
        Args:
            ranges: Array of range measurements
            angles: Array of angle measurements
            
        Returns:
            Estimated width in meters
        """
        if len(ranges) < 2:
            return 0.0
        
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        width = float(np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2))
        return width


class ConfidenceCalculator:
    """Calculates detection confidence scores"""
    
    def __init__(self, params: DetectionParams):
        """
        Initialize confidence calculator.
        
        Args:
            params: Detection parameters
        """
        self._params = params
    
    def calculate_width_score(self, width: float) -> float:
        """
        Calculate confidence score based on width (0-1).
        
        Args:
            width: Measured object width in meters
            
        Returns:
            Confidence score between 0.0 and 1.0
        """
        if width < MIN_VALID_WIDTH or not np.isfinite(width):
            return 0.0
        
        if (width < self._params.person_width_min or 
            width > self._params.person_width_max):
            return 0.0
        
        optimal_width = (self._params.person_width_min + self._params.person_width_max) / 2.0
        deviation = abs(width - optimal_width)
        max_deviation = (self._params.person_width_max - self._params.person_width_min) / 2.0
        
        return 1.0 - (deviation / max_deviation)
    
    def calculate_distance_score(self, distance: float) -> float:
        """
        Calculate confidence score based on distance (0-1).
        
        Closer objects get higher scores (better LiDAR resolution).
        
        Args:
            distance: Measured distance in meters
            
        Returns:
            Confidence score between 0.0 and 1.0
        """
        if not np.isfinite(distance):
            return 0.0
        
        if (distance < self._params.min_distance or 
            distance > self._params.max_distance):
            return 0.0
        
        range_size = self._params.max_distance - self._params.min_distance
        normalized = (distance - self._params.min_distance) / range_size
        
        return 1.0 - normalized * OPTIMAL_DISTANCE_WEIGHT
    
    @staticmethod
    def calculate_cluster_size_score(cluster_size: int) -> float:
        """
        Calculate confidence score based on cluster size.
        
        Args:
            cluster_size: Number of points in cluster
            
        Returns:
            Confidence score between 0.0 and 1.0
        """
        return min(cluster_size / MAX_CLUSTER_SIZE_NORMALIZATION, 1.0)
    
    def calculate_overall_confidence(
        self,
        width: float,
        distance: float,
        cluster_size: int
    ) -> float:
        """
        Calculate overall detection confidence.
        
        Args:
            width: Object width in meters
            distance: Object distance in meters
            cluster_size: Number of points in cluster
            
        Returns:
            Overall confidence score between 0.0 and 1.0
        """
        width_score = self.calculate_width_score(width)
        distance_score = self.calculate_distance_score(distance)
        cluster_score = self.calculate_cluster_size_score(cluster_size)
        
        confidence = (
            width_score * WIDTH_SCORE_WEIGHT +
            distance_score * DISTANCE_SCORE_WEIGHT +
            cluster_score * CLUSTER_SIZE_SCORE_WEIGHT
        )
        
        return confidence


class VisitorDetectionNode(Node):
    """ROS2 node for LiDAR-based visitor detection"""
    
    def __init__(self):
        """Initialize the visitor detection node"""
        super().__init__('visitor_detection_node')
        
        self._declare_parameters()
        self._params = self._load_parameters()
        
        self._cluster_analyzer = ClusterAnalyzer()
        self._confidence_calculator = ConfidenceCalculator(self._params)
        
        self._init_state()
        self._setup_ros_interface()
        
        self.get_logger().info('Visitor Detection Node initialized')
        self._log_configuration()
    
    def _declare_parameters(self) -> None:
        """Declare ROS2 parameters with defaults"""
        self.declare_parameter('detection_distance_min', DEFAULT_DETECTION_DISTANCE_MIN)
        self.declare_parameter('detection_distance_max', DEFAULT_DETECTION_DISTANCE_MAX)
        self.declare_parameter('detection_angle_range', DEFAULT_DETECTION_ANGLE_RANGE)
        self.declare_parameter('person_width_min', DEFAULT_PERSON_WIDTH_MIN)
        self.declare_parameter('person_width_max', DEFAULT_PERSON_WIDTH_MAX)
        self.declare_parameter('stability_threshold', DEFAULT_STABILITY_THRESHOLD)
        self.declare_parameter('visitor_timeout', DEFAULT_VISITOR_TIMEOUT)
        self.declare_parameter('cooldown_period', DEFAULT_COOLDOWN_PERIOD)
        self.declare_parameter('min_confidence', DEFAULT_MIN_CONFIDENCE)
        self.declare_parameter('lidar_frame_skip_rate', DEFAULT_FRAME_SKIP_RATE)
    
    def _load_parameters(self) -> DetectionParams:
        """
        Load and validate parameters.
        
        Returns:
            DetectionParams instance
            
        Raises:
            ValueError: If parameters are invalid
        """
        return DetectionParams(
            min_distance=self.get_parameter('detection_distance_min').value,
            max_distance=self.get_parameter('detection_distance_max').value,
            angle_range=self.get_parameter('detection_angle_range').value,
            person_width_min=self.get_parameter('person_width_min').value,
            person_width_max=self.get_parameter('person_width_max').value,
            stability_threshold=self.get_parameter('stability_threshold').value,
            visitor_timeout=self.get_parameter('visitor_timeout').value,
            cooldown_period=self.get_parameter('cooldown_period').value,
            min_confidence=self.get_parameter('min_confidence').value,
            frame_skip_rate=self.get_parameter('lidar_frame_skip_rate').value
        )
    
    def _init_state(self) -> None:
        """Initialize detection state"""
        self._detection_history: deque = deque(maxlen=self._params.stability_threshold)
        self._visitor_present: bool = False
        self._last_detection_time: float = 0.0
        self._last_visitor_left_time: float = 0.0
        self._frame_skip_counter: int = 0
    
    def _setup_ros_interface(self) -> None:
        """Setup ROS2 publishers and subscribers"""
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            sensor_qos
        )
        
        self._visitor_pub = self.create_publisher(String, 'visitor_detected', 10)
        
        self._timer = self.create_timer(1.0, self._check_visitor_timeout)
    
    def _log_configuration(self) -> None:
        """Log configuration parameters"""
        self.get_logger().info(
            f'Detection range: {self._params.min_distance}m - {self._params.max_distance}m'
        )
        self.get_logger().info(
            f'Detection angle: ±{self._params.angle_range/2}°'
        )
        self.get_logger().info(
            f'Stability threshold: {self._params.stability_threshold} frames'
        )
        self.get_logger().info(
            f'Cooldown period: {self._params.cooldown_period}s'
        )
        self.get_logger().info(
            f'Min confidence: {self._params.min_confidence}'
        )
    
    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Process LiDAR scan data to detect humans.
        
        Args:
            msg: LaserScan message
        """
        self._frame_skip_counter += 1
        if self._frame_skip_counter < self._params.frame_skip_rate:
            return
        self._frame_skip_counter = 0
        
        if not self._visitor_present and self._last_visitor_left_time > 0:
            time_since_left = time.time() - self._last_visitor_left_time
            if time_since_left < self._params.cooldown_period:
                return
        
        valid_ranges, valid_angles = self._preprocess_scan(msg)
        if valid_ranges is None:
            self._detection_history.append(False)
            return
        
        front_ranges, front_angles = self._filter_front_area(valid_ranges, valid_angles)
        if front_ranges is None:
            self._detection_history.append(False)
            return
        
        clusters = self._cluster_analyzer.find_clusters(front_ranges, front_angles)
        person_detected, best_confidence = self._analyze_clusters(clusters)
        
        self._detection_history.append(person_detected)
        self._update_detection_state(best_confidence)
    
    def _preprocess_scan(self, msg: LaserScan) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Preprocess scan data and filter invalid readings.
        
        Args:
            msg: LaserScan message
            
        Returns:
            Tuple of (valid_ranges, valid_angles) or (None, None) if no valid data
        """
        ranges = np.array(msg.ranges)
        num_points = len(ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
        
        valid_mask = (
            (ranges >= msg.range_min) &
            (ranges <= msg.range_max) &
            np.isfinite(ranges)
        )
        
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        if len(valid_ranges) == 0:
            return None, None
        
        return valid_ranges, valid_angles
    
    def _filter_front_area(
        self,
        ranges: np.ndarray,
        angles: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Filter points to front area only.
        
        Args:
            ranges: Array of range measurements
            angles: Array of angle measurements
            
        Returns:
            Tuple of (front_ranges, front_angles) or (None, None) if empty
        """
        angle_threshold = np.radians(self._params.angle_range / 2.0)
        front_mask = np.abs(angles) <= angle_threshold
        
        front_ranges = ranges[front_mask]
        front_angles = angles[front_mask]
        
        if len(front_ranges) == 0:
            return None, None
        
        return front_ranges, front_angles
    
    def _analyze_clusters(
        self,
        clusters: List[Tuple[np.ndarray, np.ndarray]]
    ) -> Tuple[bool, float]:
        """
        Analyze clusters to detect persons.
        
        Args:
            clusters: List of (ranges, angles) tuples
            
        Returns:
            Tuple of (person_detected, best_confidence)
        """
        person_detected = False
        best_confidence = 0.0
        
        for cluster_ranges, cluster_angles in clusters:
            avg_distance = float(np.mean(cluster_ranges))
            
            if not (self._params.min_distance <= avg_distance <= self._params.max_distance):
                continue
            
            width = self._cluster_analyzer.estimate_width(cluster_ranges, cluster_angles)
            
            confidence = self._confidence_calculator.calculate_overall_confidence(
                width, avg_distance, len(cluster_ranges)
            )
            
            if confidence > best_confidence:
                best_confidence = confidence
            
            width_in_range = (
                self._params.person_width_min <= width <= self._params.person_width_max
            )
            confidence_sufficient = confidence >= self._params.min_confidence
            
            if width_in_range and confidence_sufficient:
                person_detected = True
                self.get_logger().debug(
                    f'Person detected: distance={avg_distance:.2f}m, width={width:.2f}m, '
                    f'confidence={confidence:.2f}'
                )
                break
        
        return person_detected, best_confidence
    
    def _update_detection_state(self, best_confidence: float) -> None:
        """
        Update detection state based on detection history.
        
        Args:
            best_confidence: Best confidence score from current scan
        """
        if len(self._detection_history) == self._params.stability_threshold:
            stable_detection = sum(self._detection_history) >= (
                self._params.stability_threshold - 2
            )
            
            if stable_detection and not self._visitor_present:
                self._visitor_present = True
                self._last_detection_time = time.time()
                self._last_visitor_left_time = 0.0
                
                msg = String()
                msg.data = "visitor_approaching"
                self._visitor_pub.publish(msg)
                
                self.get_logger().info(
                    f'Visitor approaching detected! (confidence: {best_confidence:.2f})'
                )
            
            elif stable_detection:
                self._last_detection_time = time.time()
    
    def _check_visitor_timeout(self) -> None:
        """Check if visitor has left (timeout without detection)"""
        if self._visitor_present:
            time_since_detection = time.time() - self._last_detection_time
            
            if time_since_detection > self._params.visitor_timeout:
                self._visitor_present = False
                self._last_visitor_left_time = time.time()
                
                self._detection_history.clear()
                self.get_logger().debug('Detection history cleared after visitor left')
                
                msg = String()
                msg.data = "visitor_left"
                self._visitor_pub.publish(msg)
                
                self.get_logger().info(
                    f'Visitor left (cooldown: {self._params.cooldown_period}s)'
                )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    detector = None
    
    try:
        detector = VisitorDetectionNode()
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in visitor detection node: {e}")
    finally:
        if detector is not None:
            detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
