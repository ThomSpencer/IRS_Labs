#!/usr/bin/env python3
import math
import time
from typing import List

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

conveyorA = [1.5,-1.0,0.2]
conveyorB = [1.5, 0.0,0.0]
conveyorC = [1.5, 1.0,0.0]
conveyor_approach = [-1.0,0.0,0.0]

store = [31.0,-4.5,-math.pi/2]
store_approach = [31.0,-1.5,-math.pi/2]


movement = [0.0, 0.0, 0.0, 0.0, -1.5708, 0.0]
readyForPickup = [0.0, 0.0, 0.872665, 0.436332, 1.74533, 0.0]
pickupA = [0.0, 0.523599, 1.309, -0.349066, 1.5708, 0.296706]
pickupB = [0.0, 0.523599, 1.309, -0.349066, 1.5708, 0.296706]
pickupC = [0.0, 0.610865, 1.309, -0.401426, 1.5708, 0.0]

readyForStore = []
store = []

# --- Helper function to build a PoseStamped ---
def make_pose(x: float|int, y: float|int, yaw: float|int) -> PoseStamped:
    """
    Create a PoseStamped (position + orientation) in the 'map' frame.
    - x, y are coordinates in meters
    - yaw is robot orientation (heading) in radians
    """
    ps = PoseStamped()
    ps.header.frame_id = 'map'  # always use 'map' for navigation goals
    ps.pose.position.x = x
    ps.pose.position.y = y

    # Convert yaw (in radians) into quaternion (needed by ROS2)
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


def main():
    # 1. Initialise ROS2 and create a node
    rclpy.init()
    node = rclpy.create_node('hs_waypoint_follower_nav2pose')
    
    JOINT_NAMES: List[str] = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    PLANNING_GROUP: str = 'tmr_arm'
    ACTION_NAME: str = '/move_action'  # keep identical to your setup
    
    node.client = ActionClient(node, MoveGroup, ACTION_NAME)
    node.get_logger().info('Waiting for MoveGroup action server...')
    node.client.wait_for_server()
    node.get_logger().info('MoveGroup action server ready.')

    # 2. Create an ActionClient for the NavigateToPose action
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # --- Function to send a goal and wait for result ---
    def send_and_wait_movement(pose: PoseStamped) -> bool:
        node.get_logger().info('Waiting for Nav2 action server...')
        client.wait_for_server()

        # Update timestamp (required in headers)
        pose.header.stamp = node.get_clock().now().to_msg()

        # Wrap pose in a NavigateToPose goal message
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Simple feedback callback: prints distance left to target
        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                node.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass  # ignore if feedback doesn't have distance

        # Send the goal
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            node.get_logger().error('Goal was rejected!')
            return False

        # Wait until navigation is finished
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()

        if result is None:
            node.get_logger().error('No result returned.')
            return False

        node.get_logger().info('Goal reached successfully!')
        return True


    def send_and_wait_arm(node, joints: List[float|int]) -> bool:
        goal = _goal_from_joints(node, joints)
        send_fut = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_fut)
        handle = send_fut.result()
        if not handle or not handle.accepted:
            node.get_logger().error('Goal rejected.')
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut)
        res = res_fut.result()
        return bool(res and res.status == 4)  # 4 = STATUS_SUCCEEDED
    
    
    def _goal_from_joints(self, joints: List[float]) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5

        cs = Constraints()
        for name, val in zip(JOINT_NAMES, joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            cs.joint_constraints.append(jc)

        req.goal_constraints.append(cs)
        goal.request = req
        goal.planning_options.plan_only = False  # plan + execute
        return goal
    
    
    # --- Hard-coded waypoints for this lab. Edit this section in the code and make it your own ---
    wait_seconds = 5

    if not send_and_wait_arm(node, movement): return
    node.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at movement arm pos...')
    time.sleep(wait_seconds)
        

    wp1 = make_pose(*store_approach)          # Sample goal pose. Make your own! You can create multiple way points
    send_and_wait_movement(wp1)

    # 4. Pause to simulate "waiting at waypoint"
    node.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at waypoint 1...')
    time.sleep(wait_seconds)
    
    wp1 = make_pose(*store)          # Sample goal pose. Make your own! You can create multiple way points
    send_and_wait_movement(wp1)

    # 4. Pause to simulate "waiting at waypoint"
    node.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at waypoint 1...')
    time.sleep(wait_seconds)


    
   # --- Your custom code ends here ---

    # 6. Shutdown node and ROS2
    node.get_logger().info('Navigation sequence complete. Shutting down.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()