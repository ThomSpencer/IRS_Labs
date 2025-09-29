#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math
import time
import random

class GlobalLocalizer(Node):
    def __init__(self):
        super().__init__('global_localizer')

        # --- Publisher for robot velocity ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # --- Subscribe to AMCL pose ---
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.pose_msg = None

        # --- Subscribe to LIDAR for obstacle avoidance ---
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/virtual_hand_solo/scan',
            self.scan_callback,
            10
        )
        self.front_obstacle = False
        self.obstacle_distance_threshold = 2.5  # meters
        self.front_angles = 50  # degrees

        # Movement direction: 1 = forward, -1 = backward
        self.direction = 1

        # --- Service client for AMCL global reinit ---
        self.cli = self.create_client(Empty, '/reinitialize_global_localization')
        self.get_logger().info("Waiting for /reinitialize_global_localization service...")
        self.cli.wait_for_service()
        self.get_logger().info("Service available. Calling global localization...")

        req = Empty.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("✅ Global localization reinitialized successfully")
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")
            return

        # --- Start wiggle / localization loop ---
        self.localize_loop()

    # ---------------- Callbacks ----------------
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.pose_msg = msg

    def scan_callback(self, msg: LaserScan):
        total_angles = len(msg.ranges)
        angle_increment = math.degrees(msg.angle_increment)
        center_index = total_angles // 2
        span = int(self.front_angles / angle_increment)

        front_ranges = msg.ranges[center_index - span : center_index + span + 1]
        front_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
        self.front_obstacle = any(r < self.obstacle_distance_threshold for r in front_ranges)

    # ---------------- Helper functions ----------------
    def get_latest_pose(self):
        if self.pose_msg:
            return self.pose_msg.pose.pose
        return None

    def is_localized(self, xy_thresh=0.1, yaw_thresh=0.1):
        if self.pose_msg is None:
            return False
        cov = self.pose_msg.pose.covariance
        cov_x = cov[0]
        cov_y = cov[7]
        cov_yaw = cov[35]
        return cov_x < xy_thresh and cov_y < xy_thresh and cov_yaw < yaw_thresh

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def rotate_180(self):
        self.get_logger().info("🧱 Obstacle ahead! Performing 180° rotation...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # rad/s

        duration = math.pi / abs(twist.angular.z)  # time to rotate 180°
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration)

        while rclpy.ok() and self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.direction *= -1
        self.stop_robot()

    # ---------------- Main localization loop ----------------
    def localize_loop(self):
        self.get_logger().info("Starting localization wiggle loop...")
        rate = self.create_rate(5)  # 5 Hz decision loop

        min_travel_distance = 1.0  # meters before trusting AMCL
        distance_traveled = 0.0
        last_pose = None

        while rclpy.ok():
            pose = self.get_latest_pose()
            if pose and last_pose:
                dx = pose.position.x - last_pose.position.x
                dy = pose.position.y - last_pose.position.y
                distance_traveled += math.sqrt(dx*dx + dy*dy)
            if pose:
                last_pose = pose

            # Only accept localization after moving some distance
            if self.is_localized() and distance_traveled >= min_travel_distance:
                self.get_logger().info("🎯 Robot localized after traveling enough distance!")
                self.stop_robot()
                break

            twist = Twist()
            if self.front_obstacle:
                self.rotate_180()
                continue

            # Move in current direction with random wiggle
            twist.linear.x = 0.1 * self.direction
            twist.angular.z = random.uniform(-0.2, 0.2)
            self.cmd_pub.publish(twist)

            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.2)

        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizer()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
