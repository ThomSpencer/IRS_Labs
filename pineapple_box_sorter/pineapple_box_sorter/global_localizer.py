#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math

class GlobalLocalizer(Node):
    def __init__(self):
        super().__init__('global_localizer')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/virtual_hand_solo/scan', self.scan_callback, 10)

        self.pose_cov = None
        self.cov_history = []
        self.front_obstacle = False
        self.obstacle_distance_threshold = 2.5
        self.front_angles = 50
        self.direction = 1

        # Service to trigger localization
        self.srv = self.create_service(Trigger, 'my_service', self.handle_service)
        self.get_logger().info('Service server ready.')

        # Timer for localization steps
        self.localization_active = False
        self.localize_timer = self.create_timer(0.05, self.localize_step)  # 20 Hz

    def handle_service(self, request, response):
        self.get_logger().info('Outer service called: triggering global localization')

        # Create client for inner service
        cli = self.create_client(Empty, '/reinitialize_global_localization')

        # Asynchronous wait for service with short timeout
        if not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Inner service /reinitialize_global_localization not available")
            response.success = False
            return response

        req = Empty.Request()
        future = cli.call_async(req)

        # Add done callback to trigger localization when inner service completes
        future.add_done_callback(lambda f: self._on_global_localization_done(f))

        response.success = True  # Outer service returns immediately
        return response

    def _on_global_localization_done(self, future):
        try:
            future.result()  # Check for exceptions
            self.get_logger().info("✅ Global localization reinitialized successfully")
            self.localization_active = True  # Start localization loop
        except Exception as e:
            self.get_logger().error(f"Global localization failed: {e}")

    # --- Callbacks ---
    def pose_callback(self, msg):
        self.pose_cov = msg.pose.covariance
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

    # --- Localization check ---
    def is_localized(self, xy_thresh=0.05, yaw_thresh=0.05):
        if len(self.cov_history) < 5:
            return False
        return all(cov_x <= xy_thresh and cov_y <= xy_thresh and cov_yaw <= yaw_thresh
                   for cov_x, cov_y, cov_yaw in self.cov_history)

    # --- Motion helpers ---
    def rotate_in_place(self, angular_speed=0.4, num_turns=1):
        self.get_logger().info("♻️ Spinning 1 time/s")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed

        duration = 2 * math.pi * num_turns / abs(angular_speed)
        end_time = self.get_clock().now() + Duration(seconds=duration)

        while rclpy.ok() and self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop_robot()

    def move_forward_short(self, distance=0.3, speed=0.1):
        self.get_logger().info("➡️ Moving forwards a bit")
        twist = Twist()
        twist.linear.x = speed * self.direction
        twist.angular.z = 0.0

        duration = distance / abs(speed)
        end_time = self.get_clock().now() + Duration(seconds=duration)

        while rclpy.ok() and self.get_clock().now() < end_time:
            if self.front_obstacle:
                self.get_logger().info("🧱 Obstacle ahead, stopping short move")
                break
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop_robot()

    def rotate_180(self):
        self.get_logger().info("🧱 Obstacle ahead! Performing 180° rotation...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5

        duration = math.pi / abs(twist.angular.z)
        end_time = self.get_clock().now() + Duration(seconds=duration)

        while rclpy.ok() and self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.direction *= -1
        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # --- Timer for asynchronous localization ---
    def localize_step(self):
        if not self.localization_active:
            return

        if self.is_localized():
            self.get_logger().info("🎯 Robot localized! Stopping motion.")
            self.stop_robot()
            self.localization_active = False
            return

        if self.front_obstacle:
            self.rotate_180()
            return

        self.rotate_in_place(num_turns=1)
        self.move_forward_short(distance=0.3)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizer()
    # MultiThreadedExecutor allows inner services and timers to run without deadlock
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
