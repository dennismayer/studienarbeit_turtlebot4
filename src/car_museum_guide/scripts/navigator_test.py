#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import time
import threading

class WaypointNavigator:
    def __init__(self):
        # ROS2 initialisieren
        rclpy.init()

        # Node für Subscriber erstellen
        self.node = Node('waypoint_navigator')

        # Navigation initialisieren
        self.nav = BasicNavigator()

        self.waypoints = []
        self.initial_pose_received = False
        self.collecting_waypoints = False
        self.max_retries = 3
        self.retry_delay = 5.0
        self.nav2_active = False

        # Subscriber für Benutzereingaben
        self.initial_pose_sub = self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10
        )

        self.clicked_point_sub = self.node.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        self.node.get_logger().info('Waypoint Navigator started')

    def initial_pose_callback(self, msg):
        self.initial_pose_received = True
        self.node.get_logger().info(
            f'Initial pose received: x={msg.pose.pose.position.x:.3f}, '
            f'y={msg.pose.pose.position.y:.3f}'
        )

    def clicked_point_callback(self, msg):
        if not self.collecting_waypoints:
            return

        x = msg.point.x
        y = msg.point.y

        self.waypoints.append([x, y])
        self.node.get_logger().info(
            f'Waypoint {len(self.waypoints)} added: ({x:.3f}, {y:.3f})'
        )

    def spin_once(self):
        """Führt eine einzelne ROS2 Verarbeitungsiteration aus"""
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def wait_for_nav2_active(self, timeout=30.0):
        """Wartet auf Nav2 Aktivierung"""
        self.node.get_logger().info('Waiting for Nav2 to become active...')
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            try:
                self.nav.waitUntilNav2Active(navigator='bt_navigator', localizer='amcl')
                self.node.get_logger().info('Nav2 is ready!')
                return True
            except Exception as e:
                self.node.get_logger().info(
                    f'Still waiting for Nav2... ({e})',
                    throttle_duration_sec=5.0
                )
            time.sleep(1.0)
            self.spin_once()

        self.node.get_logger().error('Timeout waiting for Nav2 activation')
        return False

    def make_goal(self, waypoint):
        """Erstellt ein Navigationsziel"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.node.get_clock().now().to_msg()

        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        return goal

    def wait_for_initial_pose(self, timeout=60.0):
        """Wartet auf die initiale Pose vom Benutzer"""
        self.node.get_logger().info('Waiting for initial pose from RViz...')
        start_time = time.time()

        while not self.initial_pose_received:
            if (time.time() - start_time) > timeout:
                self.node.get_logger().error('Timeout waiting for initial pose')
                return False
            self.spin_once()
            time.sleep(0.1)

        return True

    def set_initial_pose_interactive(self):
        """Fordert den Benutzer auf, die initiale Pose in RViz zu setzen"""
        print("\n" + "="*60)
        print("SET INITIAL POSE IN RVIZ")
        print("="*60)
        print("1. In RViz, select the '2D Pose Estimate' tool")
        print("2. Click on the map where the robot is located")
        print("3. Drag to set the orientation")
        print("="*60)

        input("\nPress ENTER when ready to set initial pose... ")

        if not self.wait_for_initial_pose():
            self.node.get_logger().error('No initial pose received')
            return False

        self.node.get_logger().info('Initial pose set. Waiting for stabilization...')
        time.sleep(2)
        return True

    def collect_waypoints_interactive(self, num_waypoints=4):
        """Sammelt Wegpunkte interaktiv vom Benutzer"""
        print("\n" + "="*60)
        print("SELECT WAYPOINTS IN RVIZ")
        print("="*60)
        print("1. In RViz2, select the 'Publish Point' tool")
        print(f"2. Click {num_waypoints} points on the map")
        print("3. Each click will be confirmed in the terminal")
        print("="*60)

        self.collecting_waypoints = True
        self.waypoints = []

        input("\nPress ENTER to begin waypoint selection... ")

        print(f"\nNow select {num_waypoints} waypoints in RViz...")
        print("Each click will be confirmed below:\n")

        start_time = time.time()
        while len(self.waypoints) < num_waypoints:
            if (time.time() - start_time) > 120:
                self.node.get_logger().error('Timeout waiting for waypoints')
                self.collecting_waypoints = False
                return False
            self.spin_once()
            time.sleep(0.1)

        self.collecting_waypoints = False
        print(f"\nAll {num_waypoints} waypoints successfully captured!")
        return True

    def navigate_to_waypoint(self, waypoint, waypoint_num):
        """Navigiert zu einem einzelnen Wegpunkt"""
        for attempt in range(1, self.max_retries + 1):
            if attempt > 1:
                self.node.get_logger().info(
                    f'Retry {attempt}/{self.max_retries} for waypoint {waypoint_num}'
                )
                time.sleep(self.retry_delay)

            self.node.get_logger().info(
                f'Navigating to waypoint {waypoint_num}/{len(self.waypoints)}: '
                f'({waypoint[0]:.3f}, {waypoint[1]:.3f}) '
                f'[Attempt {attempt}/{self.max_retries}]'
            )

            target = self.make_goal(waypoint)
            self.nav.goToPose(target)

            while not self.nav.isTaskComplete():
                try:
                    feedback = self.nav.getFeedback()
                    if feedback and hasattr(feedback, 'distance_remaining'):
                        self.node.get_logger().info(
                            f'Distance remaining: {feedback.distance_remaining:.2f}m',
                            throttle_duration_sec=2.0
                        )
                except Exception as e:
                    self.node.get_logger().warn(f'Feedback error: {e}')

                self.spin_once()
                time.sleep(0.1)

            result = self.nav.getResult()

            if result == TaskResult.SUCCEEDED:
                self.node.get_logger().info(f'Waypoint {waypoint_num} reached successfully!')
                return True
            elif result == TaskResult.CANCELED:
                self.node.get_logger().warning(
                    f'Navigation canceled at waypoint {waypoint_num} (Attempt {attempt})'
                )
            elif result == TaskResult.FAILED:
                self.node.get_logger().warning(
                    f'Failed to reach waypoint {waypoint_num} (Attempt {attempt})'
                )

            if attempt == self.max_retries:
                self.node.get_logger().error(
                    f'Waypoint {waypoint_num} not reached after {self.max_retries} attempts'
                )
                return False

        return False

    def execute_navigation(self):
        """Führt die Navigation zu allen Wegpunkten aus"""
        self.node.get_logger().info('Starting navigation sequence...')

        successful_waypoints = 0
        failed_waypoints = 0

        for i, waypoint in enumerate(self.waypoints):
            waypoint_num = i + 1

            if self.navigate_to_waypoint(waypoint, waypoint_num):
                successful_waypoints += 1
                if waypoint_num < len(self.waypoints):
                    self.node.get_logger().info('Pausing before next waypoint...')
                    time.sleep(1.0)
            else:
                failed_waypoints += 1
                self.node.get_logger().error(f'Waypoint {waypoint_num} will be skipped')

        print("\n" + "="*60)
        print("NAVIGATION SUMMARY")
        print("="*60)
        if successful_waypoints == len(self.waypoints):
            self.node.get_logger().info(f'SUCCESS: All {successful_waypoints} waypoints reached!')
        else:
            self.node.get_logger().info(
                f'Completed: {successful_waypoints}/{len(self.waypoints)} waypoints'
            )
            if failed_waypoints > 0:
                self.node.get_logger().warning(f'{failed_waypoints} waypoint(s) failed')
        print("="*60)

    def run(self):
        """Hauptfunktion für die Navigation"""
        try:
            print("\n" + "="*60)
            print("WAYPOINT NAVIGATION SYSTEM")
            print("="*60)
            print(f"Max retries per waypoint: {self.max_retries}")
            print(f"Retry delay: {self.retry_delay}s")
            print("="*60)

            if not self.wait_for_nav2_active(timeout=30.0):
                self.node.get_logger().error('Cannot continue without Nav2')
                return False

            if not self.set_initial_pose_interactive():
                return False

            while True:
                try:
                    num_wp = input("\nHow many waypoints? (Default 4): ").strip()
                    if num_wp == "":
                        num_waypoints = 4
                    else:
                        num_waypoints = int(num_wp)
                        if num_waypoints < 1:
                            print("Please enter at least one waypoint.")
                            continue
                    break
                except ValueError:
                    print("Please enter a valid number.")

            if not self.collect_waypoints_interactive(num_waypoints):
                return False

            print("\nWaypoints captured:")
            for i, wp in enumerate(self.waypoints):
                print(f"  {i+1}. ({wp[0]:.3f}, {wp[1]:.3f})")

            print("\n" + "="*60)
            input('Press ENTER to begin navigation... ')

            self.execute_navigation()
            return True

        except Exception as e:
            self.node.get_logger().error(f'Error in navigation: {e}')
            return False

    def shutdown(self):
        """Räumt ordentlich auf"""
        try:
            self.nav.lifecycleShutdown()
            self.node.destroy_node()
            rclpy.shutdown()
        except:
            pass

def main():
    navigator = None
    try:
        navigator = WaypointNavigator()
        success = navigator.run()

        if success:
            print("\nNavigation completed successfully!")
        else:
            print("\nNavigation failed!")

    except KeyboardInterrupt:
        print("\nNavigation stopped by user")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        if navigator:
            navigator.shutdown()
        print("Waypoint Navigator shutdown complete")

if __name__ == '__main__':
    main()
