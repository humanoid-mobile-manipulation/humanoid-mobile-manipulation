# Kuavo Control Source Map

This file maps the robot-control components used by the AR-tag/KMPC
manipulation reference to the pinned Kuavo ROS application submodule. The
submodule is checked out under `kuavo_ros_application/kuavo` and provides the
reusable robot-control infrastructure. The task logic remains in
`manipulation/` and uses these upstream controllers through ROS topics and
services.

| Paper control component | Source path in the pinned submodule | Public role |
| --- | --- | --- |
| Mobile manipulation KMPC controller | `kuavo_ros_application/kuavo/src/humanoid-control/mobile_manipulator_controllers/` | Runs the model-based mobile manipulation controller and exposes the end-effector trajectory interface. |
| KMPC launch entry points | `kuavo_ros_application/kuavo/src/humanoid-control/mobile_manipulator_controllers/launch/` | Starts the controller nodes used by whole-body mobile manipulation. |
| End-effector trajectory publisher examples | `kuavo_ros_application/kuavo/src/humanoid-control/mobile_manipulator_controllers/scripts/mmTrajTest.py` and `mmTrajRushTest.py` | Demonstrates publishing target poses to `/mm/end_effector_trajectory` after enabling the mobile manipulation controller. |
| Bezier arm trajectory planning | `kuavo_ros_application/kuavo/src/humanoid-control/humanoid_plan_arm_trajectory/` | Provides Bezier trajectory messages and services for arm trajectory generation. |
| Bezier service definition | `kuavo_ros_application/kuavo/src/humanoid-control/humanoid_plan_arm_trajectory/srv/planArmTrajectoryBezierCurve.srv` | Defines the public request and response structure for Bezier arm trajectory planning. |
| Single-step position refinement | `kuavo_ros_application/kuavo/src/kuavo_msgs/srv/singleStepControl.srv` and single-step helpers under `kuavo_ros_application/kuavo/src/humanoid-control/` | Provides near-goal position refinement after long-range navigation reaches the rack-facing pose. |
| AR-tag or ArUco detection support | `kuavo_ros_application/kuavo/src/hand_eye_calibration/aruco_ros/` and `kuavo_ros_application/kuavo/src/kuavo_assets/models/apriltag/` | Provides tag detection, tag messages, and simulation assets used by structured tag localization. |
| Task-level SMT/SPS integration | `manipulation/strategy_sps.py`, `manipulation/case_sps.py`, and `manipulation/configs/` | Connects tag detections, selected target IDs, navigation goals, calibrated offsets, and manipulation commands. |

## Integration Boundary

The Kuavo ROS application submodule is an upstream robot-control dependency.
This reference documents how structured tag-localized tasks connect to it:

```text
AR-tag detection
  -> /robot_tag_info
  -> selected target IDs
  -> calibrated rack-facing navigation pose
  -> /set_nav_goal_2D
  -> near-goal single-step refinement when required
  -> KMPC or Bezier arm trajectory execution
  -> grasp and retreat sequence
```

Set rack coordinates, runtime paths, and calibration constants in the local
deployment configuration.
