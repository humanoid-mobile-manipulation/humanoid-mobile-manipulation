import numpy as np

class config:
    class common:
        """Common Configuration"""
        step_back_distance = 0.9
        enable_head_tracking = True
        half_fov = 60  # Half field of view angle, in degrees
        walk_timeout = 50  # Timeout for walking event, in seconds
        head_timeout = 30  # Timeout for head movement event, in seconds
        arm_timeout = 50  # Timeout for arm movement event, in seconds

        walk_yaw_threshold = 0.1  # Yaw angle threshold for walking event, in radians
        walk_pos_threshold = 0.15  # Position threshold for walking event, in meters

        head_search_yaws = [1, -1]  # Head search yaw angle range, in degrees
        head_search_pitchs = [-30, -15, 0, 15, 30]  # Head search pitch angle range, in degrees
        rotate_body = True  # Allow body rotation to find targets

        arm_control_mode = "fixed_base" # 'manipulation_mpc'  # Arm control mode, using Manipulation MPC control mode
        arm_pos_threshold = 0.15  # Arm position threshold, in meters
        arm_angle_threshold = np.deg2rad(20)  # Arm angle threshold, in radians

        enable_percep_when_walking = False # Enable perception while walking (look while walking)

        box_width = 0.30  # meters
        box_mass = 1.0 # kg, assuming a heavier box

        num = [0]

    class pick:
        """Pick Configuration"""
        tag_id = 1
        tag_pos_world = (0, -1, 0)  # Initial position guess, in meters, related to the negative direction of the y-axis of the machine's initial position
        tag_euler_world = (0, 0, 0)  # Initial pose guess, in Euler angles (radians)
        box_in_tag_pos = (-0.2, 0.0, 0.0)  # Box position guess in target tag, in meters
        box_in_tag_euler = (0.0, 0.0, 0.0)  # Box pose guess in target tag, in Euler angles (radians)

        stand_in_tag_pos = (0.0, 0.0, 0.6)  # Standing position guess in target tag, in meters
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # Standing pose guess in target tag, in Euler angles (radians)

        box_behind_tag = 0.12  # Distance behind tag for the box, in meters
        box_beneath_tag = 0.0  # Distance beneath tag for the box, in meters
        box_left_tag = 0.0  # Distance left of tag for the box, in meters

        force_ratio_z = 0.0
        lateral_force = 0.0  # Lateral clamping force, in N

    class place:
        """Place Configuration"""
        tag_id = 0
        tag_pos_world = (0, 1, 0)  # Placement position guess, in meters
        tag_euler_world = (0, 0, 0)  # Placement pose guess, in Euler angles (radians)
        stand_in_tag_pos = (0.0, 0.0, 0.25)  # Placement standing position guess in target tag, in meters
        stand_in_tag_euler = (-np.deg2rad(90), np.deg2rad(90), 0.0)  # Placement standing pose guess in target tag, in Euler angles (radians)

        box_behind_tag = 0.2  # Distance behind tag for the box, in meters
        box_beneath_tag = 0.5  # Distance beneath tag for the box, in meters
        box_left_tag = 0.0  # Distance left of tag for the box, in meters

        force_ratio_z = 0.0
        lateral_force = 0.0  # Lateral clamping force, in N