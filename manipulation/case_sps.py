#!/usr/bin/env python3
# coding: utf-8
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.data_type import Pose, Tag, Frame
from kuavo_humanoid_sdk.kuavo_strategy_v2.utils.logger_setup import init_logging
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.events.mobile_manipulate import (
    EventArmMoveKeyPoint, EventPercep, EventWalkToPose, EventHeadMoveKeyPoint)
from kuavo_humanoid_sdk.kuavo_strategy_v2.common.robot_sdk import RobotSDK
from kuavo_humanoid_sdk.kuavo.robot_arm import KuavoRobotArm
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import robotHandPosition

import multiprocessing
import numpy as np
import os, sys
import time
import rospy
import argparse
from typing import Optional, Union, Dict, Any, List
mother_dir = os.path.dirname(os.path.abspath(__file__))

log_path = init_logging(log_dir=os.path.join(mother_dir, "logs"), filename_prefix="grab_object_v2", enable=True)

# from configs.config_sim import config
from configs.config_real import config

from strategy_sps import (
    grab_object_and_backward,
    place_object_and_backward,
    execute_joint_trajectory,
    start_navigation,
    publish_robot_hand_position,
)
# claw = LejuClaw()
def test_pick_arm_only(nav_position):
    """
    
    """
    robot_sdk = RobotSDK()

    # 
    walk_event = EventWalkToPose(
        robot_sdk=robot_sdk,
        timeout=config.common.walk_timeout,  # Timeout for walking event, in seconds
        yaw_threshold=config.common.walk_yaw_threshold,  # Yaw angle threshold for walking event, in radians
        pos_threshold=config.common.walk_pos_threshold,  # Position threshold for walking event, in meters
        control_mode='cmd_vel'  # Use world coordinate system command position control mode
    )
    head_event = EventHeadMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.head_timeout,  # Timeout for head movement event, in seconds
    )
    percep_event = EventPercep(
        robot_sdk=robot_sdk,
        half_fov=config.common.half_fov,  # Half field of view angle, in degrees
        timeout=np.inf,  # Timeout for head movement event, in seconds
    )
    arm_event = EventArmMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.arm_timeout,  # Timeout for arm movement event, in seconds
        arm_control_mode=config.common.arm_control_mode,  # Arm control mode
        pos_threshold=config.common.arm_pos_threshold,  # Arm position threshold, in meters
        angle_threshold=config.common.arm_angle_threshold,  # Arm angle threshold, in radians
    )

    fake_target_tag = Tag(
        id=config.pick.tag_id,  # Assume target object ID is 1
        pose=Pose.from_euler(
            pos=(0.3, 0.0, 0.96),  # Initial position guess, in meters
            euler=(90, 0, -90),  # Initial pose guess, in Euler angles (radians)
            frame=Frame.ODOM,  # Use odometry coordinate system
            degrees=True
        )
    )


    success = grab_object_and_backward(
        robot_sdk=robot_sdk,
        arm_event=arm_event,
        step_back_distance=config.common.step_back_distance,  # Distance to step back after grabbing, in meters
        nav_position=nav_position,
    )
    # robot_sdk.control.arm_reset()
    # if not success:
    #     print("Failed to grab the target object, exiting strategy.")
    #     return False
    time.sleep(2)
    # Publish grab message for dexterous hand
    left_hand_cmd = [0,0,0,0,0,0] # Replace with actual hand control commands
    # right_hand_cmd = [100, 100, 100, 100, 100, 100] 
    right_hand_cmd = [0, 0, 0, 0, 0, 0] 
    
    publish_robot_hand_position(left_hand_cmd, right_hand_cmd)
    time.sleep(1)
    target_poses = [
        [2,[0,0,0,0,0,0,0,-50.409,-40.698,50,-100.3,12.796,24.66,30.25]],
        ]
    execute_joint_trajectory(robot_sdk=robot_sdk,target_poses=target_poses)
    time.sleep(2)
    # robot_sdk.control.arm_reset()

def test_place_arm_only():
    """
    Test the functionality of using only the arm.
    """
    robot_sdk = RobotSDK()

    # Initialize events
    walk_event = EventWalkToPose(
        robot_sdk=robot_sdk,
        timeout=config.common.walk_timeout,  # Timeout for walking event, in seconds
        yaw_threshold=config.common.walk_yaw_threshold,  # Yaw angle threshold for walking event, in radians
        pos_threshold=config.common.walk_pos_threshold,  # Position threshold for walking event, in meters
        control_mode='cmd_vel'  # Use world coordinate system command position control mode
    )
    head_event = EventHeadMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.head_timeout,  # Timeout for head movement event, in seconds
    )
    percep_event = EventPercep(
        robot_sdk=robot_sdk,
        half_fov=config.common.half_fov,  # Half field of view angle, in degrees
        timeout=np.inf,  # Timeout for head movement event, in seconds
    )
    arm_event = EventArmMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.arm_timeout,  # Timeout for arm movement event, in seconds
        arm_control_mode=config.common.arm_control_mode,  # Arm control mode
        pos_threshold=config.common.arm_pos_threshold,  # Arm position threshold, in meters
        angle_threshold=config.common.arm_angle_threshold,  # Arm angle threshold, in radians
    )

    fake_target_tag = Tag(
        id=config.pick.tag_id,  # Assume target object ID is 1
        pose=Pose.from_euler(
            pos=(0.3, 0.0, 0.96),  # Initial position guess, in meters
            euler=(90, 0, -90),  # Initial pose guess, in Euler angles (radians)
            frame=Frame.ODOM,  # Use odometry coordinate system
            degrees=True
        )
    )

    success = place_object_and_backward(
        walk_event=walk_event,
        arm_event=arm_event,
        object_width=config.common.object_width,
        object_behind_tag=config.place.object_behind_tag,  # Distance behind tag, in meters
        object_beneath_tag=config.place.object_beneath_tag,  # Distance beneath tag, in meters
        object_left_tag=config.place.object_left_tag,  # Distance left of tag, in meters
        tag=fake_target_tag,
        step_back_distance=config.common.step_back_distance,  # Distance to step back after placing, in meters

        object_mass=config.common.object_mass,  # Assume object mass, in kg, used to calculate longitudinal wrench
        force_ratio_z=config.place.force_ratio_z,  # Empirical coefficient (based on 1.5kg corresponding to 5N: 5/(1.5*9.8)≈0.34
        lateral_force=config.place.lateral_force,  # Lateral clamping force, in N
        enable_backward=False
    )

    robot_sdk.control.arm_reset()


def grab_one_object(
                    arm_event:EventArmMoveKeyPoint, 
                    user_input=True, 
                    use_safe_arm_control=True,
                    nav_position=(-0.25,0.19,0),
                    place_position=(-3.19,0.09,3.14),
                    ):
    """
    Execute the complete strategy for grabbing an object.

    Parameters:
        user_input (bool): Whether to wait for user input to continue each step.
        use_safe_arm_control (bool): Whether to use the safe event system for arm control,
                                   True=use event system (recommended), False=use original SDK direct call.

    Returns:
        bool: Whether the strategy was successfully completed.
    """
  
    # Initialize robot
    robot_sdk = RobotSDK()
  
    # Raise arm
    target_poses = [
        # [1,[-10.0, 0.0, 0.0, -50.0, 0.0, 0.0, 0.0,20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0]],
        [2,[0,0,0,0,0,0,0,-69.409,-59.698,51,-100.3,12.796,24.66,30.25]]   
    ]
    execute_joint_trajectory(robot_sdk,target_poses)
    time.sleep(1)
    if user_input:
        input("Ready to grab point, press Enter to continue... \n")
    print("🔵 Step: Start approaching the target...")

    # Raise head
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(0))
    # Grab position
    success = start_navigation(*nav_position) # SLAM position for grabbing
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")
    if not success:
        print("Failed to approach the target object, exiting strategy.")
        return False
    
    # Lower head by 20 degrees
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(25))
    time.sleep(3)
    print("======================================================")
    if user_input:
        input("Ready to grab point, intervene target detection grab, press Enter to continue... \n")

    # Start intervening target detection grab
    success = grab_object_and_backward(
        robot_sdk=robot_sdk,
        arm_event=arm_event,
        step_back_distance=config.common.step_back_distance,  # Distance to step back after grabbing, in meters
        nav_position=nav_position,
    )
    # robot_sdk.control.arm_reset()
    
    # Publish grab message for dexterous hand
    left_hand_cmd = [0,0,0,0,0,0] # Replace with actual hand control commands
    right_hand_cmd = [100, 100, 100, 100, 100, 100]  # Replace with actual hand control commands
    
    success = publish_robot_hand_position(left_hand_cmd, right_hand_cmd)
    time.sleep(1)
    target_poses = [
        [2,[0,0,0,0,0,0,0,-69.409,-40.698,50,-100.3,12.796,24.66,30.25]],
        ]
    execute_joint_trajectory(robot_sdk=robot_sdk,target_poses=target_poses)
    time.sleep(0.5)
    ## Add keyboard event here, press specific key to continue
    print("====================Grabbed the target object==================================")


    # Retreat after grabbing
    success = start_navigation(-0.75,0.19,0, is_keep_yaw=True) # SLAM position for retreating
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")
    if not success:
        print("Failed to retreat the target object, exiting strategy.")
        return False
    if user_input:
        input("Ready to grab point, press Enter to continue... \n")
    time.sleep(3)
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(0))
    # Place
    success = start_navigation(*place_position) # SLAM position for placing
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")
    if not success:
        print("Failed to reach the placement position, exiting strategy.")
        return False
    success = robot_sdk.control.stance()
    if not success:
        print("Failed to stand, exiting strategy.")
        return False
    time.sleep(2)
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(25))
    time.sleep(1)
    # Place arm joints
    target_poses = [
        [1,[0,0,0,0,0,0,0,-65.409,-20.698,71,-80.3,12.796,24.66,20.25]],
        # [2,[0,0,0,0,0,0,0,-69.409,-30.698,50,-80.3,12.796,24.66,30.25]],  
    ]
    execute_joint_trajectory(robot_sdk,target_poses)
    time.sleep(2)


    # Release dexterous hand publish message
    left_hand_cmd = [0,0,0,0,0,0] # Replace with actual hand control commands
    right_hand_cmd = [0, 0, 0, 0, 0, 0]  # Replace with actual hand control commands
    
    success = publish_robot_hand_position(left_hand_cmd, right_hand_cmd)
    time.sleep(2)

    target_poses = [
        # [1,[0,0,0,0,0,0,0,-65.409,-20.698,71,-80.3,12.796,24.66,20.25]],
        [2,[0,0,0,0,0,0,0,-69.409,-30.698,50,-80.3,12.796,24.66,30.25]],  
    ]
    execute_joint_trajectory(robot_sdk,target_poses)
    time.sleep(2)
    # Retreat after placing
    success = start_navigation(-2.69,0.09,3.14,is_keep_yaw=True) # SLAM position for retreating after placing
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")
    if not success:
        print("Failed to place the target object, exiting strategy.")
        return False
    time.sleep(1)
    # Reset arm
    
    robot_sdk.control.control_head(yaw=np.deg2rad(0), pitch=np.deg2rad(0))
    return True


def get_nav_position_for_tag(tag_id: int, 
                             operation_type: str = "pick") -> Optional[Union[tuple, dict]]:
    """
    Get the corresponding navigation position based on tag_id and operation type
    
    According to the navigation position dictionary defined in the configuration file, 
    return the corresponding navigation position for different tag_id and operation types.
    Supports pick and place operation types.
    
    Args:
        tag_id: Target tag ID
        operation_type: Operation type, optional "pick" or "place", default is "pick"
        
    Returns:
        For pick operation returns tuple: (x, y, yaw), if not configured returns default position (1.0, 2.0, 0.0)
        For place operation returns dict or None (if not found)
        
    Examples:
        >>> get_nav_position_for_tag(5)  # Default pick operation
        (1.2, 2.1, 0.0)
        >>> get_nav_position_for_tag(8, "place")
        {'x': 0.9, 'y': 1.8, 'yaw': 0.0}
    """
    default_pick_position = (1.0, 2.0, 0.0)
    
    try:
        if operation_type == "pick":
            if hasattr(config, 'pick') and hasattr(config.pick, 'pick_nav_positions'):
                if tag_id in config.pick.pick_nav_positions:
                    nav_position = config.pick.pick_nav_positions[tag_id]
                    print(f"Pick operation - Tag {tag_id} uses navigation position: {nav_position}")
                    return nav_position
            print(f"Pick operation - Tag {tag_id} uses default navigation position: {default_pick_position}")
            return default_pick_position
            
        elif operation_type == "place":
            if hasattr(config, 'place') and hasattr(config.place, 'place_nav_positions'):
                position = config.place.place_nav_positions[tag_id]
                if position:
                    print(f"Place operation - Tag {tag_id} uses navigation position: {position}")
                    return position
            print(f"Place operation - No navigation position found for tag {tag_id}")
            return None
            
        else:
            print(f"Invalid operation type: {operation_type}, only 'pick' or 'place' are supported")
            return None if operation_type == "place" else default_pick_position
            
    except Exception as e:
        print(f"Error occurred while getting {operation_type} navigation position for Tag {tag_id}: {e}")
        return None if operation_type == "place" else default_pick_position

def main():
    # Create argument parser
    parser = argparse.ArgumentParser(description='Robot grabbing operation program')
    
    # Add parameters - Use nargs=3 to group three coordinates as one parameter
    parser.add_argument('--start_position', type=float, nargs=3, required=True, 
                       metavar=('X', 'Y', 'THETA'), help='Starting position coordinates (x y theta)')
    parser.add_argument('--pick_position', type=float, nargs=3, required=True, 
                       metavar=('X', 'Y', 'THETA'), help='Pick position coordinates (x y theta)')
    parser.add_argument('--loop_count', type=int, default=1, help='Number of loops, default is 5')
    
    # Parse parameters
    args = parser.parse_args()
    
    # Directly convert parameters to tuple
    start_nav_position = tuple(args.start_position)
    print("start_nav_position:",start_nav_position)
    pick_position = tuple(args.pick_position)
    print("pick_position:",pick_position)
    
    robot_sdk = RobotSDK()
    robot_arm=KuavoRobotArm()
    arm_event = EventArmMoveKeyPoint(
        robot_sdk=robot_sdk,
        timeout=config.common.arm_timeout,  # Timeout for arm movement event, in seconds
        arm_control_mode=config.common.arm_control_mode,  # Arm control mode
        pos_threshold=config.common.arm_pos_threshold,  # Arm position threshold, in meters
        angle_threshold=config.common.arm_angle_threshold,  # Arm angle threshold, in radians
    )
    # Statistics
    successful_picks = 0
    failed_picks = 0

    # Set arm control mode
    # robot_arm.set_external_control_arm_mode()
    # init_arm_trajectory(robot_sdk)

    place_position = get_nav_position_for_tag(0, "place")
    
    success = start_navigation(*start_nav_position) # Initial SLAM position
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")
    if not success:
        print("Failed to approach the starting position, exiting strategy.")
        raise SystemExit("Program exited")
    time.sleep(1)
    for i in range(args.loop_count):
        res = False  # Initialize result variable
        try:
            print("###########pick_position", pick_position) 
            res = grab_one_object(
                arm_event=arm_event,
                user_input=False,
                use_safe_arm_control=True,
                nav_position=pick_position,
                place_position=place_position,
            )
            
            if res:
                successful_picks += 1
                print(f"✅  {i} Grab successful")
            else:
                failed_picks += 1
                print(f"❌  {i} Grab failed")
            
            print(f"### Case {i}/{args.loop_count} ended ###")
            
        except Exception as e:
            failed_picks += 1
            print(f"❌ Error occurred during processing: {e}")
            print(f"### Case {i}/{args.loop_count} ended with exception ###")
        
        if not res:
            print(f"\n⚠️ Grab failed, stopping further processing")
            break
    
    robot_sdk.control.arm_reset()
    success = start_navigation(*start_nav_position)  # Return to initial SLAM position
    rospy.loginfo(f"Navigation started: {'successfully' if success else 'failed'}")

    # Output statistics
    print(f"\n{'='*50}")
    print(f"Successful grabs: {successful_picks}")
    print(f"Failed grabs: {failed_picks}")
    print(f"{'='*50}")

if __name__ == "__main__":
    main()
# python3 case_sps.py --start_position -1.2 0.19 0 --pick_position -0.25 0.24 0 --loop_count 2

   
    ########################Pick arm test#####################
    # test_pick_arm_only((-0.25,0.19,0))
    ########################Place arm test#####################
    # for eps in config.common.num:
    #     test_place_arm_only()
