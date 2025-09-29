#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math
import time

class GlobalLocalizer(Node):
    def __init__(self):
        super().__init__('global_localizer')

        # Publisher for robot velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # Subscribe to AMCL pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.pose_cov = None
        self.cov_history = []

        # Subscribe to LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/virtual_hand_solo/scan',
            self.scan_callback,
            10
        )
        self.front_obstacle = False
        self.obstacle_distance_threshold = 2.5
        self.front_angles = 50

        # Direction flag: 1 = forward, -1 = backward
        self.direction = 1

        # Service client for global localization
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

        # Start the localization loop
        self.localize_loop()

    def pose_callback(self, msg):
        self.pose_cov = msg.pose.covariance

        # Keep last 5 covariance readings to check stability
        cov_x = self.pose_cov[0]
        cov_y = self.pose_cov[7]
        cov_yaw = self.pose_cov[35]
        self.cov_history.append((cov_x, cov_y, cov_yaw))
        if len(self.cov_history) > 5:
            self.cov_history.pop(0)

    def scan_callback(self, msg):
        total_angles = len(msg.ranges)
        angle_increment = math.degrees(msg.angle_increment)
        center_index = total_angles // 2
        span = int(self.front_angles / angle_increment)

        front_ranges = msg.ranges[center_index - span : center_index + span + 1]
        front_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
        self.front_obstacle = any(r < self.obstacle_distance_threshold for r in front_ranges)

    def is_localized(self, xy_thresh=0.05, yaw_thresh=0.05):
        # Require multiple stable readings to consider localized
        if len(self.cov_history) < 5:
            return False
        for cov_x, cov_y, cov_yaw in self.cov_history:
            if cov_x > xy_thresh or cov_y > xy_thresh or cov_yaw > yaw_thresh:
                return False
        return True

    def rotate_in_place(self, angular_speed=0.4, num_turns=1):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed

        duration = 2 * math.pi * num_turns / abs(angular_speed)
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration)

        self.get_logger().info(f"🔄 Rotating in place for {num_turns} turn(s)")
        while rclpy.ok() and self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.stop_robot()

    def move_forward_short(self, distance=0.3, speed=0.1):
        twist = Twist()
        twist.linear.x = speed * self.direction
        twist.angular.z = 0.0

        duration = distance / abs(speed)
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration)

        self.get_logger().info(f"➡️ Moving forward {distance} m")
        while rclpy.ok() and self.get_clock().now() < end_time:
            if self.front_obstacle:
                self.get_logger().info("🧱 Obstacle ahead, stopping short move")
                break
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.stop_robot()

    def rotate_180(self):
        self.get_logger().info("🧱 Obstacle ahead! Performing 180° rotation...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # rad/s

        duration = math.pi / abs(twist.angular.z)
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=duration)

        while rclpy.ok() and self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        self.direction *= -1
        self.stop_robot()

    def localize_loop(self):
        self.get_logger().info("Starting improved localization loop...")
        while rclpy.ok():
            if self.is_localized():
                self.get_logger().info("🎯 Robot localized! Stopping motion.")
                self.stop_robot()
                break

            # If obstacle ahead, rotate 180
            if self.front_obstacle:
                self.rotate_180()
                continue

            # Step 1: rotate in place to scan surroundings
            self.rotate_in_place(num_turns=1)

            # Step 2: move forward/backward a short distance
            self.move_forward_short(distance=0.3)

        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizer()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
