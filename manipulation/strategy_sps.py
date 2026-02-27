#!/usr/bin/env python3
# coding: utf-8
import time
from typing import List, Dict, Any, Tuple
import numpy as np
import math
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Int32MultiArray
import multiprocessing

from configs.config_real import config

from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Tag, Pose, Frame, Transform3D
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.base_event import EventStatus
from kuavo_humanoid_sdk.kuavo.core.core import KuavoRobotCore
from kuavo_humanoid_sdk.kuavo.core.ros.tools import KuavoRobotToolsCore
from kuavo_humanoid_sdk.kuavo.robot_arm import KuavoRobotArm
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot,KuavoRobotState,LejuClaw
from kuavo_humanoid_sdk.kuavo.dexterous_hand import DexterousHand
from kuavo_humanoid_sdk.interfaces.data_types import KuavoManipulationMpcCtrlMode, KuavoManipulationMpcFrame, KuavoPose
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest, controlLejuClawResponse
from geometry_msgs.msg import Twist
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import robotHandPosition

from dynamic_biped.srv import GetTargetPartPoseInCamera
from nav_straight_walk.srv import SetPose2D,SetPose2DRequest
from std_srvs.srv import Trigger,TriggerRequest
# Strategy writing principles:
#
# 1. No hidden state transfer between strategies: Strategies should not have hidden state transfers. All variables must be explicitly passed. This ensures that each strategy can be independently enabled and tested by constructing inputs.
# 2. Reuse of event instances: Event instances can be reused but cannot transfer state across strategies. Each strategy should manage its own event state.
# 3. Abstraction of events:
#    Events should include three stages: start, process, and termination, and return a clear state at the end.
#    Events can be blocking (exclusive resource during execution until completion) or non-blocking (allowing concurrent execution with other events).
#    If non-blocking, multiple events may run simultaneously.
# 4. Example: For instance, the "move to find Tag" strategy may consist of three events: move event, perception event, and head movement event, which work together to complete the task.
# 5. Testability of events: Each event should be testable independently because it has well-defined targets (Target) and clear inputs and outputs

def control_leju_claw(position, velocity=[50, 50], effort=[1.0, 1.0], 
                    claw_names=['left_claw', 'right_claw'], 
                    service_name='/control_robot_leju_claw'):
    """
    Elegant wrapper function to control Leju claw
    
    Args:
        position: Claw position list [left_pos, right_pos]
        velocity: Claw velocity list [left_vel, right_vel] (default [50, 50])
        effort: Claw effort list [left_effort, right_effort] (default [1.0, 1.0])
        claw_names: Claw name list (default ['left_claw', 'right_claw'])
        service_name: Service name (default '/control_robot_leju_claw')
    """
    try:
        # Ensure service is available
        rospy.wait_for_service(service_name, timeout=1.0)
        
        # Create request message
        claw_control_msg = controlLejuClawRequest()
        claw_control_msg.data.name = claw_names
        claw_control_msg.data.position = position
        claw_control_msg.data.velocity = velocity
        claw_control_msg.data.effort = effort
        
        # Call service
        service_proxy = rospy.ServiceProxy(service_name, controlLejuClaw)
        response = service_proxy(claw_control_msg)
        return response
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None
    except rospy.ROSException as e:
        rospy.logerr(f"Service wait timeout: {e}")
        return None

# Predefined common actions
CLAW_OPEN = [30, 40]
CLAW_PRE = [30, 40]
CLAW_CLOSE = [100, 40]


class PoseController:
    def __init__(self):
        self.cmd_pose_pub = rospy.Publisher('/cmd_pose', Twist, queue_size=10)
    
    def _create_pose_msg(self, linear_x=0.0, linear_y=0.0, linear_z=0.0,
                        angular_x=0.0, angular_y=0.0, angular_z=0.0):
        """
        Create and return a Twist message
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        return msg
    
    def bend_down(self, angular_y=0.2):
        """
        Bend down action
        """
        pose_msg = self._create_pose_msg(angular_y=angular_y)
        self.cmd_pose_pub.publish(pose_msg)
        rospy.loginfo("Executing bend down action")
    
    def stand_up(self):
        """
        Stand up action
        """
        pose_msg = self._create_pose_msg()  # Default values are all 0
        self.cmd_pose_pub.publish(pose_msg)
        rospy.loginfo("Executing stand up action")


def start_navigation(x, y, theta, is_keep_yaw=False,use_final_adjustment=False,use_user_set_reach_goal_safe_region_radius=False,reach_goal_safe_region_radius=0.8):
    """
    Start navigation to the specified 2D pose
    
    Args:
        x (float): Target x position
        y (float): Target y position
        theta (float): Target orientation in radians
        
    Returns:
        bool: True if navigation started successfully, False otherwise
    """
    try:
        set_nav_goal_2d_srv_name = '/set_nav_goal_2D'
        set_goal_client = rospy.ServiceProxy(set_nav_goal_2d_srv_name, SetPose2D)
        req = SetPose2DRequest()
        req.pose.x = x
        req.pose.y = y
        req.pose.theta = theta
        req.is_keep_yaw = is_keep_yaw
        req.use_final_adjustment = use_final_adjustment
        req.use_user_set_reach_goal_safe_region_radius = use_user_set_reach_goal_safe_region_radius
        req.reach_goal_safe_region_radius = reach_goal_safe_region_radius
        resp = set_goal_client(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

def stop_navigation():
    """
    Cancel current navigation task
    
    Returns:
        bool: True if cancellation was successful, False otherwise
    """
    try:
        stop_nav_goal_2d_srv_name = '/stop_nav_goal_2D'
        cancel_goal_client = rospy.ServiceProxy(stop_nav_goal_2d_srv_name, Trigger)
        req = TriggerRequest()
        resp = cancel_goal_client(req)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False
    
def move_backward(nav, step_back_distance=0.6,direction='pick', is_keep_yaw=True):
    """
    Move backward for pick or place operation.
    
    Args:
        nav: Target navigation coordinates (x, y, yaw)
        direction: 'pick' to subtract step_back_distance, 'place' to add
        is_keep_yaw: Whether to maintain the yaw orientation
    """
    if direction == 'pick':
        new_x = nav[0] - step_back_distance
    elif direction == 'place':
        new_x = nav[0] + step_back_distance
    else:
        rospy.logerr(f"Invalid direction: {direction}. Use 'pick' or 'place'")
        return False
        
    success = start_navigation(new_x, nav[1], nav[2], is_keep_yaw=is_keep_yaw)
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")
    return success


def pick_move_arm_and_backward(
        tag_info,
        robot_sdk: RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        arm_traj: Tuple[List[Pose], List[Pose]],  # Stores left and right arm data, frame can be odom or bask_link
        step_back_distance: float,  # Distance to move backward, in meters
        tag: Tag = None,
        arm_wrench: Tuple[List, List] = None,  # Optional arm torque data, stores left and right arm torques
        enable_backward: bool = True,
        nav_position: Tuple[List, List] = None,
        
):
    """
    Pick up the box while moving backward.

    Args:
        walk_event (EventWalkToPose): Walking event.
        arm_event (EventArmMoveKeyPoint): Arm movement event.
        arm_traj (Tuple[List[Pose], List[Pose]]): Arm trajectory, stores left and right arm data.
        step_back_distance (float): Distance to move backward, in meters.
        tag (Tag): Optional target tag.
        arm_wrench (Tuple[List, List]): Optional arm torque data.

    Returns:
        bool: Whether the operation was successfully completed.
    """
  

    print("✅ Successfully moved the arm, starting to move backward...")
    arm_event.close()
    # Close the claw
    time.sleep(2)
    control_leju_claw(position=CLAW_CLOSE)
    time.sleep(0.5)
    # 3. Retract the arm to the chest and observe the arm
    chest_left_pose = [Pose.from_euler(pos=(tag_info[1].x-0.3,tag_info[1].y-0.03,tag_info[1].z+0.1), euler=(90, 0.0, -90), degrees=True,
                            frame=Frame.BASE),
                       Pose.from_euler(pos=(tag_info[1].x-0.3,tag_info[1].y-0.28,tag_info[1].z), euler=(90, -60.0, -160), degrees=True,
                            frame=Frame.BASE),
    ]
    
    
    chest_right_pose = [Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                            frame=Frame.BASE),
                        Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                        frame=Frame.BASE),
    ]
    chest_traj = (chest_left_pose, chest_right_pose)

    arm_event.open()
    if not arm_event.set_target(chest_traj, tag=tag):
        print("❌ Failed to set arm retraction key point")
        return False

    while True:
        arm_status = arm_event.step()
        if arm_status != EventStatus.RUNNING:
            break

    arm_event.close()
   
    # Lower the head by 15 degrees
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(20))
    time.sleep(1)
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(0))
    print("Strategy completed.")
    return True

def place_move_arm_and_backward(
        robot_sdk: RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        arm_traj: Tuple[List[Pose], List[Pose]],  # Stores left and right arm data, frame can be odom or bask_link
        step_back_distance: float,  # Distance to move backward, in meters
        arm_wrench: Tuple[List, List] = None,  # Optional arm torque data, stores left and right arm torques
        enable_backward:bool = True,
        palce_position:Tuple[List, List] = None,
    ):
    """
    Place the box while moving backward.

    Args:
        walk_event (EventWalkToPose): Walking event.
        arm_event (EventArmMoveKeyPoint): Arm movement event.
        arm_traj (Tuple[List[Pose], List[Pose]]): Arm trajectory, stores left and right arm data.
        step_back_distance (float): Distance to move backward, in meters.
        tag (Tag): Optional target tag.
        arm_wrench (Tuple[List, List]): Optional arm torque data.

    Returns:
        bool: Whether the operation was successfully completed.
    """

    robot_arm=KuavoRobotArm()
    robot_arm.set_external_control_arm_mode()

    target_poses = [
            [0.5,[-55.94858079892865, 11.847661356046677, -59.1096426277084, -64.08723682515728, -19.72022572676684, -8.202121533739984, -20.115263527957676, 27.047374428164584, -11.406112946986038, 11.485358748703637, -56.491676386860256, -13.801707640205462, -1.090192867404783, 8.369241324429169]],
            [0.8,[-58.98050409807267, -12.534475294258618, -67.54631403402183, -17.813915836411784, -59.49755270480513, 19.461068839622815, -24.524301346229002, 25.500536212327944, -7.994605846140557, 12.113954126616326, -51.104789637359964, -11.3372125973276, -3.6256630188787886, 7.54137061053416]],
            [1.1,[-56.032079760328266, -10.26299731183272, -63.05054309117076, -15.809181042588943, -50.15786382975109, -7.583794710486194, -27.906737495740078, 24.703313302082087, -7.70169820310629, 12.249448409425739, -49.07978604226111, -11.744624012484252, -2.908910763404843, 6.466916894875008]],
            [1.3,[-56.14388067814826, -10.302724231699866, -62.923698126952004, -16.181655080033618, -49.86720549779491, -9.029820922958445, -27.759823330253422, 24.5898632204952, -7.718481518867492, 12.225287051030929, -48.762837899467435, -11.986386943106142, -2.829542433395776, 6.420255707589033]],
    ]
    execute_joint_trajectory(robot_sdk,target_poses)
    time.sleep(1.3)
    
    controller = PoseController()
    controller.bend_down()
    time.sleep(1)
    
    control_leju_claw(position=CLAW_PRE)
    time.sleep(2)
 
    controller.stand_up()
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(0))
   
    target_poses = [
        [0.5,[-39.27617921954182, 17.997447111892647, -62.841195160255516, -76.58595611913377, -47.67752686969816, -17.101593937274576, -13.604045248634351, 28.37473957217686, -11.61968640824094, 13.29641323799504, -56.7288858586432, -14.791012982210678, -2.1104951005887584, 14.649980319544332]],
        [0.8,[-33.78311343500147, 30.951001494065544, -62.47462031689757, -93.02179520676026, -52.939489786854345, -6.1110060617817, -18.186876679043042, 25.932420613986647, -10.528017585292913, 12.913447000141948, -51.20313352296793, -15.1006473776624, -1.8161850854426604, 13.363724133347173]],
        [1.5,[-13.952884112360803, 36.7013380328393, -54.99100461768642, -99.3959396323488, -47.93220757519881, 6.41706015074737, -9.210062245312939, 23.31851548842856, -9.138297240801418, 11.900506030002946, -46.78555898109102, -14.979767056540439, -1.638198907259003, 11.195696839799277]],
    ]
    execute_joint_trajectory(robot_sdk,target_poses)
    time.sleep(0.8)
    return True


def grab_object_and_backward(
        robot_sdk: RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        step_back_distance: float,  # Distance to move backward, in meters
        target_tag_id: int=None,
        tag: Tag = None,
        arm_wrench: Tuple[List, List] = None,
        enable_arm_control: bool = True,  # Whether to enable arm and dexterous hand control
        enable_backward: bool = True,
        nav_position: Tuple[List, List] = None,
) -> bool:
    """
    Grab the box and move backward.

    Args:
        walk_event (EventWalkToPose): Walking event.
        arm_event (EventArmMoveKeyPoint): Arm movement event.
        step_back_distance (float): Distance to move backward, in meters.
        tag (Tag): Target tag.
        force_ratio_z (float): Longitudinal force empirical coefficient.
        lateral_force (float): Lateral clamping force, in N.
        enable_arm_control (bool): Whether to enable arm and dexterous hand control, default True.

    Returns:
        bool: Whether the operation was successfully completed.
    """
 
    # If arm control is not enabled, directly perform the backward operation
    if not enable_arm_control:
        print("🔵 Skipping arm and dexterous hand control, directly performing backward operation...")
        move_backward(nav_position,step_back_distance,direction='pick',is_keep_yaw=True)
        return True
    # =================== Calculate the arm poses (Pose) for each key point =================== #
    start = time.time()
    p, q =  get_target_pose_with_retry()
    print("Service time",time.time()-start)
    print("🚗🚗🚗🚗🚗🚗🚗🚗🚗position\n",p)
    if True:
        pick_left_arm_poses = [
            Pose.from_euler(pos=(0.004345, 0.292643,  -0.300229), euler=(0, 0, 0), degrees=True,
                            frame=Frame.BASE),
            ]
        pick_right_arm_poses = [
            Pose.from_euler(pos=(p.x+0.085, p.y-0.05, p.z-0.12), euler=(-77, -5, 108), degrees=True,
                            frame=Frame.BASE),
            ]

        arm_traj = (pick_left_arm_poses, pick_right_arm_poses)
        arm_event.open() 
        # Open arm event
        if not arm_event.set_target(arm_traj, arm_wrench=arm_wrench, tag=tag):
            print("❌ Failed to set arm key point")
            return False
        while True:
            arm_status = arm_event.step()
            if arm_status != EventStatus.RUNNING:
                break
        if arm_status != EventStatus.SUCCESS:
            print("❌ Arm movement failed, exiting strategy.")
            arm_event.close()
            return False
        arm_event.close()
        return True

def place_object_and_backward(
        robot_sdk:RobotSDK,
        arm_event: EventArmMoveKeyPoint,
        step_back_distance: float,  # Distance to move backward, in meters
        tag: Tag,  # Optional target tag, used to obtain position and orientation information

        object_width: float,
        object_behind_tag: float,  # Distance of the box behind the tag, in meters
        object_beneath_tag: float,  # Distance of the box below the tag, in meters
        object_left_tag: float,  # Distance of the box to the left of the tag, in meters

        object_mass: float,  # Assumed box mass, in kg, used to calculate longitudinal wrench
        force_ratio_z: float,  # Empirical coefficient (based on 1.5kg corresponding to 5N: 5/(1.5*9.8)≈0.34
        lateral_force: float,  # Lateral clamping force, in N
        enable_arm_control: bool = True,  # Whether to enable arm and dexterous hand control, default False.
        enable_backward:bool = True,
        palce_position:Tuple[List, List] = None,
):
    """
    Place the box and move backward.

    Args:
        walk_event (EventWalkToPose): Walking event.
        arm_event (EventArmMoveKeyPoint): Arm movement event.
        step_back_distance (float): Distance to move backward, in meters.
        tag (Tag): Target tag.
        object_width (float): Box width.
        object_behind_tag (float): Distance of the box behind the tag, in meters.
        object_beneath_tag (float): Distance of the box below the tag, in meters.
        object_left_tag (float): Distance of the box to the left of the tag, in meters.
        object_mass (float): Box mass, in kg.
        force_ratio_z (float): Longitudinal force empirical coefficient.
        lateral_force (float): Lateral clamping force, in N.
        enable_arm_control (bool): Whether to enable arm and dexterous hand control, default False.

    Returns:
        bool: Whether the operation was successfully completed.
    """

    # If arm control is not enabled, directly perform the backward operation
    if not enable_arm_control:
        print("🔵 Skipping arm and dexterous hand control, directly performing backward operation...")
        
        move_backward(palce_position,step_back_distance,direction='place',is_keep_yaw=True)
        return True
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(20))
    # =================== Calculate the arm poses (Pose) for each key point =================== #

    place_left_arm_poses = [
        Pose.from_euler(pos=(0.8, -0.207935,  -0.05), euler=(-90, -200.0, 10), degrees=True,
                        frame=Frame.BASE),
        Pose.from_euler(pos=(0.8, -0.107935,  -0.2), euler=(40, -30.0, -150), degrees=True,
                        frame=Frame.BASE),
    ]
    place_right_arm_poses = [
        Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                        frame=Frame.BASE),
        Pose.from_euler(pos=(0.004345, -0.292643,  -0.270229), euler=(0.0374, -14.5817, 0.0205), degrees=True,
                        frame=Frame.BASE),
    ]  # Arm key point data, assumed to be an empty list


    arm_traj = (place_left_arm_poses, place_right_arm_poses)
    success = place_move_arm_and_backward(robot_sdk, arm_event, arm_traj, step_back_distance, tag=tag ,enable_backward=enable_backward,palce_position=palce_position)

    return success

def execute_joint_trajectory(robot_sdk: RobotSDK, target_poses: list):
    """
    Execute a joint trajectory on the robot arm.
    
    Args:
        robot_sdk: Instance of RobotSDK
        target_poses: List of target poses, where each pose is a list in format [time, [joint_angles_in_degrees]]
                      Example: [[1.0, [20, 0, 0, -30, 0, 0, 0, 20, 0, 0, -30, 0, 0, 0]],
                               [2.5, [30, 10, 5, -25, 5, 5, 5, 25, 5, 5, -25, 5, 5, 5]]]
    """
    if not target_poses:
        print("Error: target_poses is empty!")
        return False
    
    times = [pose[0] for pose in target_poses]
    
    # Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]
    
    print("🚀🚀🚀🚀🚀🚀")
    if not robot_sdk.arm.control_arm_joint_trajectory(times, q_frames):
        print("control_arm_joint_trajectory failed!")
        return False
    
    return True

def subscribe_and_print_tag_info(target_tag_id=None):
    """
    Subscribe to the '/robot_tag_info_smt' topic and print the detected tag pose information.
    If target_tag_id is specified, only return information for that ID.

    Args:
        target_tag_id: Target tag ID, if None, print all tag information

    Returns:
        tuple: If the target tag is found, return (tag_id, position, orientation), otherwise return None
    """
    if rospy.get_node_uri() is None:  # If the node is not initialized
        rospy.init_node('tag_info_subscriber', anonymous=True)
    try:
        msg = rospy.wait_for_message('/robot_tag_info', AprilTagDetectionArray, timeout=5.0)
        for detection in msg.detections:
            tag_id = detection.id[0] if detection.id else "Unknown"
            if tag_id == target_tag_id:
                print("QR code coordinates:",tag_id, detection.pose.pose.pose.position, detection.pose.pose.pose.orientation)
                return (tag_id, detection.pose.pose.pose.position, detection.pose.pose.pose.orientation)
    except rospy.ROSException:
        print("Timeout while waiting for tag message")
    return None

def wait_tag_ids(color):
    """
    Block until receiving a list of IDs
    return  -> list[int]
    """
    if rospy.get_node_uri() is None:  # If the node is not initialized
        rospy.init_node('tag_info_subscriber', anonymous=True)
    topic = f"/{color}_tag_ids"          # -> /red_tag_ids or /blue_tag_ids
    rospy.loginfo(f"[grasp] Waiting for {topic} …")
    msg = rospy.wait_for_message(topic, Int32MultiArray)
    if not msg.data:
        rospy.logerr(f"[grasp] {topic} Empty list!")
        raise RuntimeError("empty tag id list")
    rospy.loginfo(f"[grasp] Grasping order ({color}): {list(msg.data)}")
    return list(msg.data)

def control_arm_joint_trajectory_0(robot_sdk: RobotSDK):
    target_poses = [
        [0.5, [-10.0, 0.0, 0.0, -50.0, 0.0, 0.0, 0.0,
            20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0]],
    ]
   
    times = [pose[0] for pose in target_poses]
    
    # !!! Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]
    print("🚀🚀🚀🚀🚀🚀")
    if not robot_sdk.arm.control_arm_joint_trajectory(times, q_frames):
        print("control_arm_joint_trajectory failed!")


pub_ctrl_robot_hand = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
def publish_robot_hand_position(left_position: list, right_position: list) -> bool:
    """
    Publish robot hand position control message
    
    Args:
        left_position: Left hand position command list
        right_position: Right hand position command list
        
    Returns:
        bool: True if published successfully, False otherwise
    """
    try:
        
        # Create message object
        hand_pose_msg = robotHandPosition()
        
        # Convert list to bytes type (according to message structure requirements)
        hand_pose_msg.left_hand_position = bytes(left_position)
        hand_pose_msg.right_hand_position = bytes(right_position)
        
        # Publish message
        pub_ctrl_robot_hand.publish(hand_pose_msg)
        
        rospy.logdebug(f"Published robot hand position: left={left_position}, right={right_position}")
        return True
        
    except Exception as e:
        rospy.logerr(f"Failed to publish robot hand position: {e}")
        return False


def call_get_target_part_pose():
    rospy.wait_for_service('/get_target_part_pose_in_camera')
    try:
        proxy = rospy.ServiceProxy('/get_target_part_pose_in_camera', GetTargetPartPoseInCamera)
        resp = proxy()  # No request parameters
        if not resp.success:
            rospy.logwarn("Service returned failure: %s", resp.message)
            return None
        pose = resp.pose_in_camera  # Actually the pose in odom
        return pose, resp.message
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None 

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    
    Args:
        x, y, z, w: Four components of the quaternion
        
    Returns:
        (roll, pitch, yaw): Tuple of Euler angles (in radians)
    """
    # Calculate roll (rotation around x-axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Calculate pitch (rotation around y-axis)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees, sign matches sinp
    else:
        pitch = math.asin(sinp)
    
    # Calculate yaw (rotation around z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw  # Return a tuple containing three values

def get_target_pose_with_retry(max_attempts=5):
    """
    Attempt to get the target pose, retrying up to max_attempts times
    
    Args:
        max_attempts: Maximum number of attempts, default is 5
        
    Returns:
        On success, returns a tuple (p, euler), where p is the position and euler is a tuple of Euler angles (roll, pitch, yaw)
        On failure, returns None
    """
    # Initialize the node (if not already initialized)
    try:
        rospy.init_node("get_target_pose_client", anonymous=True)
    except rospy.exceptions.ROSException:
        # Node already initialized, ignore exception
        pass
    
    attempt = 0
    while attempt < max_attempts:
        attempt += 1
        rospy.loginfo(f"Attempting to get target pose (Attempt {attempt})...")
        
        result = call_get_target_part_pose()
        
        if result is None:
            rospy.logwarn(f"Attempt {attempt} failed to get pose or service call failed")
            # If not the last attempt, wait for a while before retrying
            if attempt < max_attempts:
                rospy.sleep(1)  # Wait 1 second before retrying
            continue
        
        pose, msg = result
        p = pose.position
        q = pose.orientation
        
        # Convert quaternion to Euler angles
        euler_angles = quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        rospy.loginfo(f"Attempt {attempt} successfully got pose")
        rospy.loginfo(f"message: {msg}")
        rospy.loginfo(f"position [m]: {p.x}, {p.y}, {p.z}")
        rospy.loginfo(f"orientation [roll pitch yaw in radians]: {euler_angles[0]}, {euler_angles[1]}, {euler_angles[2]}")
        
        return p, euler_angles
    
    rospy.logerr(f"Failed to get pose after {max_attempts} attempts")
    return None

def init_arm_trajectory(robot_sdk: RobotSDK):
    """
    Initialize the default joint trajectory of the robot arm.

    This function defines a preset arm posture trajectory and calls the underlying control interface to execute it.
    It is mainly used before the robot starts or the task begins to move the arm to a standard initial posture,
    ensuring the stability and repeatability of subsequent actions.

    Args:
        robot_sdk (RobotSDK): Robot SDK instance, used to call the underlying arm control interface.

    Returns:
        None: No explicit return value. If control fails, an error message will be output to the console.
    """
    target_poses = [
        [0.5, [21.573113, 8.605678, -20.807334, -47.349897, -9.777089, 12.313784, -9.395159, 24.291423, -8.738348, 11.655110, -46.388633, -15.214493, -1.107452, 7.228860]],
    ]
   
    times = [pose[0] for pose in target_poses]
    
    # !!! Convert degrees to radians
    q_frames = [[math.radians(angle) for angle in pose[1]] for pose in target_poses]
    
    if not robot_sdk.arm.control_arm_joint_trajectory(times, q_frames):
        print("control_arm_joint_trajectory failed!")


if __name__ == '__main__':
    start = time.time()
    result = get_target_pose_with_retry()
    print("Service time",time.time()-start)
