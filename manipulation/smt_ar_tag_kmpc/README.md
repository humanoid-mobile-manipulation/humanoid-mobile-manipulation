# SMT AR-Tag and KMPC Manipulation Pipeline

This directory documents the paper-specific SMT manipulation pipeline. It is a
sanitized public reference for the code paths used in the manuscript and does
not include private deployment credentials, internal image links, or factory
site-specific calibration values.

## Paper Mapping

The SMT task in the manuscript uses a structured environment. The target part
or tray is localized with AR tags, the robot navigates to a calibrated standing
pose near the rack, and the arm follows a model-based trajectory generated for
KMPC execution.

| Paper component | Public repository location | Role |
| --- | --- | --- |
| AR-tag visual localization | `kuavo_ros_application/kuavo/src/hand_eye_calibration/aruco_ros/`, `kuavo_ros_application/kuavo/src/ros_vision/detection_apriltag/` when available in the checked-out Kuavo tree | Detects rack/tray tags and publishes tag detections. |
| Light-selected target ID | `/red_tag_ids`, `/blue_tag_ids` topics consumed by `manipulation/strategy_sps.py` | Converts the active indicator light into the tag ID order to process. |
| Navigation to calibrated stand pose | `manipulation/strategy_sps.py:start_navigation`, `manipulation/configs/config_real.py` | Sends the robot to the pre-calibrated rack-facing pose before manipulation. |
| Point-LIO/GLIM/NDT navigation | `navigation/point_lio_glim_ndt/` | Documents the public navigation pipeline used before SMT manipulation. |
| AR-tag pose query | `manipulation/strategy_sps.py:subscribe_and_print_tag_info` | Reads the detected tag pose for the selected target. |
| KMPC/mobile manipulation controller | `kuavo_ros_application/kuavo/src/humanoid-control/mobile_manipulator_controllers/` | Provides the model-based mobile manipulation controller used by the SMT pipeline. |
| Kuavo control source map | `manipulation/smt_ar_tag_kmpc/kuavo_control_source_map.md` | Maps the pinned control submodule to KMPC, Bezier trajectory planning, single-step refinement, and tag detection sources. |
| Task-level sequencing | `manipulation/case_sps.py` and strategy helpers in `manipulation/strategy_sps.py` | Coordinates navigation, perception, arm motion, grasping, and retreat actions. |

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

The exact launch files, branch names, host credentials, Docker image locations,
and factory-specific stand poses are intentionally omitted from this public
repository. Those values are deployment artifacts rather than paper-specific
algorithmic code.

## Runtime Checks

For a public reproduction environment, the SMT pipeline is considered connected
when the following checks pass:

- `/robot_tag_info` publishes the target rack or tray tag pose;
- `/red_tag_ids` or `/blue_tag_ids` publishes the selected target ID when the
  indicator-light module is enabled;
- `/T_w_from_b` publishes the robot base pose used for stand-pose calibration;
- `/set_nav_goal_2D` accepts the calibrated rack-facing navigation target;
- `/stop_nav_goal_2D` cancels an active navigation task for safety or task
  transition;
- the KMPC or Bezier trajectory controller accepts the arm or end-effector
  trajectory selected by the task-level strategy.

## Calibration Data To Provide For Reproduction

For a new lab setup, reproduce the SMT pipeline by calibrating:

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
