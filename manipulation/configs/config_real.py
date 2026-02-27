import numpy as np

class config:
    class common:
        """Common Configuration"""
        step_back_distance = 0.2
        enable_head_tracking = True
        half_fov = 60  # Half field of view angle, in degrees
        walk_timeout = 50  # Timeout for walking event, in seconds
        head_timeout = 30  # Timeout for head movement event, in seconds
        arm_timeout = 50  # Timeout for arm movement event, in seconds

        walk_yaw_threshold = 0.10  # Yaw angle threshold for walking event, in radians
        walk_pos_threshold = 0.10  # Position threshold for walking event, in meters

        head_search_yaws = [15, -15]  # Head search yaw angle range, in degrees
        head_search_pitchs = [-30, -15, 0, 15, 30]  # Head search pitch angle range, in degrees
        # head_search_pitchs = [-15, 0, 15]
        rotate_body = True  # Allow body rotation to find targets

        arm_control_mode = "fixed_base" # 'manipulation_mpc'  # Arm control mode, using Manipulation MPC control mode
        arm_pos_threshold = 0.1  # Arm position threshold, in meters
        arm_angle_threshold = np.deg2rad(20)  # Arm angle threshold, in radians

        enable_percep_when_walking = False # Enable perception while walking (look while walking)

        object_width = 0.30  # meters
        object_mass = 1.0 # kg, assuming a heavier box

        num = [4]

    class pick:
        """Pick Configuration"""
        tag_id = 2
        tag_pos_world = (1, 0, 0)  # Initial position guess, in meters
        tag_euler_world = (0, 0, 0)  # Initial pose guess, in Euler angles (radians)
        object_in_tag_pos = (-0.2, 0.0, 0.0)  # Box position guess in target tag, in meters
        object_in_tag_euler = (0.0, 0.0, 0.0)  # Box pose guess in target tag, in Euler angles (radians)

        stand_in_tag_pos = (0.66, 0.0, 0.65)  # Standing position guess in target tag, in meters
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # Standing pose guess in target tag, in Euler angles (radians)

        object_behind_tag = 0.13  # Distance behind tag for the box, in meters
        object_beneath_tag = -0.04  # Distance beneath tag for the box, in meters
        object_left_tag = 0.0  # Distance left of tag for the box, in meters

        force_ratio_z = 0.34
        lateral_force = 10.0  # Lateral clamping force, in N
        

        # Different tag_id corresponding navigation position adjustments
        # Format: (x, y, theta), target position for SLAM navigation
        pick_nav_positions = {
              
            0: (-0.25,0.19,0),    # Navigation position for tag 0
            # 1: (-0.75,0.19,0),    # Navigation position for tag 1
          
        }

        squat_tag = [] # Tag IDs to squat, empty means no squatting
        
        # Squatting and standing configuration parameters
        squat_height_pick = -0.15  # Squatting height for picking, in meters (positive value means squatting)
        stand_height_pick = 0.15  # Standing height for placing, in meters (negative value means standing)
        motion_delay = 2.0  # Waiting time after action completion, in seconds

    class place:
        """Place Configuration"""
        tag_id = 1
        tag_pos_world = (-1, 0, 0)  # Placement position guess, in meters
        tag_euler_world = (0, 0, 0)  # Placement pose guess, in Euler angles (radians)
        stand_in_tag_pos = (0.0, 0.0, 0.4)  # Placement standing position guess in target tag, in meters
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # Placement standing pose guess in target tag, in Euler angles (radians)

        object_behind_tag = -0.05  # Distance behind tag for the box, in meters
        object_beneath_tag = 0.60  # Distance beneath tag for the box, in meters
        object_left_tag = 0.0  # Distance left of tag for the box, in meters

        force_ratio_z = 0.34
        lateral_force = 10.0  # Lateral clamping force, in N

        place_nav_positions = {
            0: (-3.19,0.09,3.14),
            # 1: (-2.69,0.09,3.14),
        }
