# AR-Tag and KMPC Manipulation Reference

This directory documents the AR-tag localization and KMPC manipulation
reference used by the structured task workflow.

## Pipeline Components

The structured task workflow localizes the target part or tray with AR tags,
navigates the robot to a calibrated standing pose near the rack, and executes a
model-based arm trajectory with KMPC.

| Component | Repository location | Role |
| --- | --- | --- |
| AR-tag visual localization | `kuavo_ros_application/kuavo/src/hand_eye_calibration/aruco_ros/`, `kuavo_ros_application/kuavo/src/ros_vision/detection_apriltag/` when available in the checked-out Kuavo tree | Detects rack/tray tags and publishes tag detections. |
| Light-selected target ID | `/red_tag_ids`, `/blue_tag_ids` topics consumed by `manipulation/strategy_sps.py` | Converts the active indicator light into the tag ID order to process. |
| Navigation to calibrated stand pose | `manipulation/strategy_sps.py:start_navigation`, `manipulation/configs/config_real.py` | Sends the robot to the pre-calibrated rack-facing pose before manipulation. |
| Point-LIO/GLIM/NDT navigation | `navigation/point_lio_glim_ndt/` | Documents the shared navigation pipeline used before manipulation. |
| AR-tag pose query | `manipulation/strategy_sps.py:subscribe_and_print_tag_info` | Reads the detected tag pose for the selected target. |
| KMPC/mobile manipulation controller | `kuavo_ros_application/kuavo/src/humanoid-control/mobile_manipulator_controllers/` | Provides the model-based mobile manipulation controller used by the structured task pipeline. |
| Kuavo control source map | `manipulation/smt_ar_tag_kmpc/kuavo_control_source_map.md` | Maps the pinned control submodule to KMPC, Bezier trajectory planning, single-step refinement, and tag detection sources. |
| Task-level interfaces | `manipulation/strategy_sps.py`, `manipulation/configs/`, and local task scripts | Connect navigation, perception, arm motion, grasping, and retreat actions. |

## Public Execution Outline

1. Start the camera and AR-tag detection stack on the upper computer. The
   project used both NUC/RealSense-style and Orin/Orbbec-style deployments, but
   the public release only depends on the resulting tag-detection topics.
2. Confirm that tag detections are published on `/robot_tag_info`.
3. Confirm that the active light target IDs are published on `/red_tag_ids` or
   `/blue_tag_ids`.
4. Start the SLAM/localization stack and command-velocity bridge.
5. Start the lower-body controller and mobile manipulation controller.
6. Run the task-level strategy script after all upstream topics and services are
   ready.

## Runtime Checks

For a deployment environment, the structured task pipeline is considered
connected when the following checks pass:

- `/robot_tag_info` publishes the target rack or tray tag pose;
- `/red_tag_ids` or `/blue_tag_ids` publishes the selected target ID when the
  indicator-light module is enabled;
- `/T_w_from_b` publishes the robot base pose used for stand-pose calibration;
- `/set_nav_goal_2D` accepts the calibrated rack-facing navigation target;
- `/stop_nav_goal_2D` cancels an active navigation task for safety or task
  transition;
- the KMPC or Bezier trajectory controller accepts the arm or end-effector
  trajectory selected by the task-level strategy.

## Calibration Inputs

For a new lab setup, reproduce the structured task pipeline by calibrating:

- camera intrinsics and camera-to-robot extrinsics;
- AR-tag family, AR-tag ID, physical tag size, and tag-to-part offset;
- rack-facing standing pose for each workstation, represented as `(x, y, yaw)`;
- pre-grasp, grasp, and retreat offsets for the chosen end-effector;
- KMPC controller parameters for the target robot configuration.

## Interface Summary

The public runtime interface is:

```text
camera image/depth
  -> AR-tag detector
  -> /robot_tag_info
  -> selected target IDs from /red_tag_ids or /blue_tag_ids
  -> navigation stand pose from manipulation/configs/config_real.py
  -> /set_nav_goal_2D
  -> KMPC/mobile manipulation controller
  -> grasp and retreat sequence
```

Use `smt_pipeline_interfaces.py` for a compact list of the relevant topics,
services, and source files.
