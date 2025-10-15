#!/usr/bin/env python3
import math
import time
from typing import List
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState
from std_msgs.msg import String


# --- Hard-coded positions ---
conveyorA_pos = [2.5, -1.5, math.pi/2]
conveyorB_pos = [2.5, -1.0, math.pi/2]
conveyorC_pos = [2.5, 2.0, math.pi/2]
conveyor_approach_pos = [-1.0, 0.0, 0.0]

store_pos = [31.0, -5.25, -math.pi/2]
post_store_pos = [31.0, -5.25, math.pi]
store_approach_pos = [31.0, -1.5, -math.pi/2]

# --- Hard-coded arm angles (radians) ---
movement_arm = [0.0, 0.0, 0.0, 0.0, -1.5708, 0.0]
home_arm = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
readyForPickup_arm = [0.0, 0.0, 0.872665, 0.436332, 1.74533, 0.0]
pickupA_arm = [-math.pi/2, 0.623599, 1.209, -0.349066, 1.5708, 0.296706]
pickupB_arm = [-math.pi/2, 0.623599, 1.209, -0.349066, 1.5708, 0.296706]
pickupC_arm = [-math.pi/2, 0.710865, 1.209, -0.401426, 1.5708, 0.0]

readyForStore_arm = [0.0, 0.0, 1.39626, 0.0, 1.72788, 0.0]
store_arm = [0.0, 0.349066, 1.309, 0.0, 1.5708, 0.0]
post_store_arm = [0.0, -math.pi/6, 5*math.pi/9, 0.0, math.pi/2, 0.0]

JOINT_NAMES: List[str] = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
PLANNING_GROUP: str = 'tmr_arm'
MOVE_ACTION_NAME: str = '/move_action'


# --- Helper function ---
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.pose.position.x = x
    ps.pose.position.y = y
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


class WaypointFollower(Node):
    
    def __init__(self):
        super().__init__('hs_waypoint_follower_nav2pose')
        # --- MoveGroup ActionClient ---
        self.arm_client = ActionClient(self, MoveGroup, MOVE_ACTION_NAME)
        self.get_logger().info('Waiting for MoveGroup action server...')
        self.arm_client.wait_for_server()
        self.get_logger().info('MoveGroup action server ready.')

        # --- Nav2 ActionClient ---
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.PLC_sub = self.create_subscription(String, 'hmi/unified_status', self.listener_callback, 10)
        self.box_location  = None
        
    def listener_callback(self, msg):
        jsonData = json.loads(msg.data)
        boxWeight = jsonData['box']['weight_raw']
        boxLocation = jsonData['box']['location'][-1]  # Last character only
        
        if boxWeight == '\n' or boxWeight == '' or boxLocation == '-':
            self.box_location = None 
            return  # Ignore empty messages
        
        
        boxWeight = boxWeight.split()[1].strip()
        prev_status = self.box_location
        #self.get_logger().info(f"Box weight: {boxWeight}, Box location: {boxLocation}")
        
        if int(boxWeight) > 10 and boxLocation == 'A':
            # Big Box
            self.box_location = "big"
        elif int(boxWeight) < 6 and boxLocation == 'C':
            # Small Box
            self.box_location = "sml"
        elif 6 <= int(boxWeight) <= 10 and boxLocation == 'B':
            # Medium Box
            self.box_location = 'med'
        else:
            self.box_location = None  # Unknown weight, reset box location
            
        if prev_status != self.box_location:
            self.get_logger().info(f'Box status updated: {self.box_location}')

    # --- Send MoveIt arm goal ---
    def send_and_wait_arm(self, joints: List[float]) -> bool:
        goal = self._goal_from_joints(joints)
        send_fut = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_fut)
        handle = send_fut.result()

        if not handle or not handle.accepted:
            self.get_logger().error('Arm goal rejected.')
            return False

        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        res = res_fut.result()

        if res is None:
            self.get_logger().error('Arm result is None.')
            return False

        self.get_logger().info(f"Arm result status: {res.status}")
        return res.status == 4  # STATUS_SUCCEEDED

    # --- Build MoveGroup goal ---
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
            cs.joint_constraints.append(jc) # type: ignore

        req.goal_constraints.append(cs) # type: ignore
        goal.request = req
        goal.planning_options.plan_only = False
        return goal

    # --- Send navigation goal ---
    def send_and_wait_movement(self, pose: PoseStamped) -> bool:
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()

        pose.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # def feedback_cb(fb):
        #     try:
        #         dist = fb.feedback.distance_remaining
        #         self.get_logger().info(f'Distance remaining: {dist:.2f} m')
        #     except Exception:
        #         pass

        send_future = self.nav_client.send_goal_async(goal)#feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result is None:
            self.get_logger().error('Navigation result None')
            return False

        self.get_logger().info('Navigation goal reached.')
        return True

    # --- Run the waypoint sequence ---
    def run_sequence(self):
        wait_seconds = 2

        # --- Precalculate poses ---
        a_pos = make_pose(*conveyorA_pos)
        b_pos = make_pose(*conveyorB_pos)
        c_pos = make_pose(*conveyorC_pos)
        conv_appr = make_pose(*conveyor_approach_pos)
        drop_pos = make_pose(*store_pos)
        pos_drop_pos = make_pose(*post_store_pos)
        drop_appr = make_pose(*store_approach_pos)

        self.get_logger().info('Starting navigation sequence...')
        if not self.send_and_wait_arm(movement_arm): raise Exception("ARM Planner error 1")
        self.send_and_wait_movement(conv_appr)
        rclpy.spin_once(self, timeout_sec=wait_seconds) 

        box = None
        while self.box_location is None:
            self._logger.info('waiting for box location...')
            rclpy.spin_once(self, timeout_sec=2.0)
        box = self.box_location

        self.get_logger().info(f'Box detected: {box}')
    
        
        try:
        #if box == 'big':
            self.get_logger().info('Big box detected, moving to conveyor A.')
            if not self.send_and_wait_arm(pickupA_arm): raise Exception("ARM Planner error 2")
            self.send_and_wait_movement(a_pos)
            
        #elif box == 'med':
            self.get_logger().info('Medium box detected, moving to conveyor B.')
            if not self.send_and_wait_arm(pickupB_arm): raise Exception("ARM Planner error 2")
            self.send_and_wait_movement(b_pos)
            
        #elif box == 'sml':
            self.get_logger().info('Small box detected, moving to conveyor C.')
            if not self.send_and_wait_arm(pickupC_arm): raise Exception("ARM Planner error 2")
            self.send_and_wait_movement(c_pos)
        except Exception as e:
            pass        
        
        #input("Waiting...")
        rclpy.spin_once(self, timeout_sec=wait_seconds) 
        
        if not self.send_and_wait_arm(movement_arm): raise Exception("ARM Planner error 3")
        self.send_and_wait_movement(drop_appr)
        
        if not self.send_and_wait_arm(readyForStore_arm): raise Exception("ARM Planner error 4")
        rclpy.spin_once(self, timeout_sec=wait_seconds) 

        self.send_and_wait_movement(drop_pos)
        if not self.send_and_wait_arm(store_arm): raise Exception("ARM Planner error 5")
        rclpy.spin_once(self, timeout_sec=wait_seconds) 
        
        if not self.send_and_wait_arm(post_store_arm): raise Exception("ARM Planner error 5.1")
        rclpy.spin_once(self, timeout_sec=wait_seconds) 
        self.send_and_wait_movement(pos_drop_pos)
        #self.send_and_wait_movement(drop_appr) Robot doesnt reverse well


        if not self.send_and_wait_arm(movement_arm): raise Exception("ARM Planner error 6")
        self.send_and_wait_movement(conv_appr)
        if not self.send_and_wait_arm(home_arm): raise Exception("ARM Planner error 7")

        self.get_logger().info('Navigation sequence complete.')
        
        


def main():
    rclpy.init()
    node = WaypointFollower()
    inp = None
    try:
        while inp == None or inp == '\n' or inp == '':
            node.run_sequence()
            inp = input("Press enter to go again: ")
    except Exception as e:
        node.get_logger().error(f'Sequence aborted: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
