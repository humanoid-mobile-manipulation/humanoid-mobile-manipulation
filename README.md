# Humanoid Mobile Manipulation Repository

This repository is part of our ongoing research work, which is currently under submission. It contains the implementation of various components for humanoid mobile manipulation tasks. Below is an overview of the repository structure and its purpose:

## Repository Structure

### Paper-specific module map

| Paper module | Repository location | Notes |
| --- | --- | --- |
| SPS YOLOv12/RandLA-Net perception pipeline | `perception/sps_yolo_randla/` | RGB-D capture, data preparation, YOLO segmentation training, candidate-mask score labeling, RandLA-Net scorer training, and whole-image inference reference. |
| SMT AR-tag/KMPC manipulation pipeline | `manipulation/smt_ar_tag_kmpc/` | Sanitized public reference for AR-tag localization, light-selected target IDs, calibrated navigation stand poses, and KMPC/mobile manipulation integration. |
| Point-LIO/GLIM/NDT navigation pipeline | `navigation/point_lio_glim_ndt/` | Sanitized public reference and runtime templates for LiDAR-inertial localization, NDT scan matching, 2D mapping/navigation, navigation pose calibration, and dynamic-obstacle halt logic. |
| SPS manipulation strategy | `manipulation/` | Task-level SPS strategy scripts and real/simulation configuration. |
| Kuavo ROS control and manipulation infrastructure | `kuavo_ros_application/` | Robot SDK, ROS messages/services, demos, navigation/control infrastructure, and deployment utilities. |
| Imitation-learning framework | `imitation_learning_framework/` | LeRobot/Kuavo data conversion, policy training, and deployment examples. |

The SPS perception code under `perception/sps_yolo_randla/` is the main
paper-specific addition for the YOLO/RandLA-Net pipeline. Datasets and trained
weights are not included in this repository; the scripts document the training
and deployment workflow used by the project.

The perception pipeline is separate from `imitation_learning_framework/`: it
selects the target-part candidate used by downstream manipulation or
imitation-learning policy code, but it is not itself an IL policy-training
module.

For review and reproducibility checks, the table below maps the main
paper-specific subsystems to the public code or sanitized integration reference
in this repository.

## Reviewer Concern Map

| Reviewer concern | Public artifact | Status |
| --- | --- | --- |
| YOLOv12/RandLA-Net perception pipeline | `perception/sps_yolo_randla/` | Released as training, labeling, scoring, inference, and ROS deployment reference scripts. |
| Candidate mask score labeling and RandLA-Net training | `perception/sps_yolo_randla/mask_score/` | Released as reusable scripts with historical defaults documented in the module README. |
| Point-LIO/GLIM/NDT navigation pipeline | `navigation/point_lio_glim_ndt/` | Released as a sanitized public integration reference plus runtime templates for MID360 startup, LiDAR SLAM, 2D mapping, 2D navigation, stop navigation, and relocalization score checks. |
| Dynamic-obstacle halt implementation | `navigation/point_lio_glim_ndt/scripts/start_2d_navigation.sh` and `navigation/point_lio_glim_ndt/dynamic_obstacle_halt_monitor.py` | Released as the obstacle-detection startup switch used by straight navigation plus a testable stop-zone monitor with optional ROS point-cloud adapter. |
| SMT AR-tag/KMPC manipulation pipeline | `manipulation/smt_ar_tag_kmpc/`, `manipulation/strategy_sps.py`, and `manipulation/smt_ar_tag_kmpc/kuavo_control_source_map.md` | Released as public interface mapping plus task-level strategy code. |
| KMPC/mobile manipulation controller | `kuavo_ros_application/kuavo/src/humanoid-control/mobile_manipulator_controllers/` | Present in the pinned Kuavo ROS application submodule. |
| Bezier arm trajectory and single-step refinement support | `kuavo_ros_application/kuavo/src/humanoid-control/humanoid_plan_arm_trajectory/`, `kuavo_ros_application/kuavo/src/kuavo_msgs/srv/singleStepControl.srv`, and `manipulation/smt_ar_tag_kmpc/kuavo_control_source_map.md` | Present in the pinned Kuavo ROS application submodule and mapped to the SMT task-level integration. |
| imitation-learning framework | `imitation_learning_framework/IL/` | Present as the LeRobot/Kuavo data, training, and deployment framework. |

## Reproduction Overview

The public reproduction path is intentionally organized by paper subsystem
rather than by internal deployment machine. This keeps the release useful for
review while avoiding private runtime images, credentials, site paths, and
factory calibration values.

1. Initialize the repository submodules so that the Kuavo control tree and the
   imitation-learning framework are available.
2. For SPS, use `perception/sps_yolo_randla/` to collect RGB-D data, train the
   YOLO segmentation model, label candidate-mask scores, train the RandLA-Net
   scorer, and run whole-image candidate selection.
3. For SMT, use `manipulation/smt_ar_tag_kmpc/` to identify the AR-tag,
   selected target ID, navigation stand pose, KMPC controller, Bezier
   trajectory planner, and task-level strategy interfaces.
4. For navigation, use `navigation/point_lio_glim_ndt/` to configure the
   LiDAR-inertial SLAM/localization stack, record rack-facing stand poses, run
   2D navigation, and enable dynamic-obstacle halt logic.
5. For imitation learning, use `imitation_learning_framework/IL/` for data
   conversion, policy training, and deployment examples. The SPS perception
   pipeline can provide target candidates to downstream manipulation or policy
   code, but it is not itself an IL training module.

The README files in each module are concise public guides. They document what
to run, which topics/services connect the modules, and which local calibration
values a new deployment must provide. They do not duplicate private lab
runbooks with account names, download links, fixed paths, production map files,
or real workstation coordinates.

### 1. `imitation_learning_framework/`
This directory contains the imitation learning (IL) framework. It includes configurations, training scripts, and utilities for implementing and evaluating imitation learning algorithms. Key components include:
- `configs/`: Configuration files for datasets and environments.
- `kuavo_data/`: Scripts and utilities for data processing and dataset management.
- `kuavo_deploy/`: Deployment scripts and examples for real-world and simulated environments.
- `kuavo_train/`: Training scripts and utilities for policy learning.
- `third_party/`: External libraries and dependencies.

### 2. `kuavo_ros_application/`
This directory contains the architecture for the robot used in this research. It includes modules for navigation, control, and other essential functionalities. Key components include:
- `docker/`: Docker-related scripts for setting up the environment.
- `docs/`: Documentation for robot setup, motion control APIs, and other related topics.
- `src/`: Source code for robot-specific functionalities.

### 3. `manipulation/`
This directory contains the code for manipulation tasks. It includes:
- `case_sps.py`: Specific case implementations for manipulation.
- `strategy_sps.py`: Strategies for manipulation tasks.
- `configs/`: Configuration files for real and simulated environments.
- `smt_ar_tag_kmpc/`: Paper-specific SMT AR-tag/KMPC pipeline mapping and public interface reference. Sensitive deployment credentials, internal artifact links, and factory-specific calibration values are intentionally excluded.

### 4. `perception/`
This directory contains paper-specific perception code. Key components include:
- `sps_yolo_randla/data_collection/`: RGB-D capture for RealSense and Orbbec cameras.
- `sps_yolo_randla/data_preparation/`: Utilities for reorganizing raw RGB/depth captures.
- `sps_yolo_randla/yolo_segmentation/`: Ultralytics YOLO segmentation training entry point.
- `sps_yolo_randla/mask_score/`: Candidate-mask score labeling, dataset conversion, and RandLA-Net score training/testing code.
- `sps_yolo_randla/inference/`: Whole-image segmentation and mask-score inference reference.
- `sps_yolo_randla/deployment_reference/`: ROS deployment reference for connecting perception output to the downstream grasping service.

### 5. `navigation/`
This directory contains paper-specific navigation references:
- `point_lio_glim_ndt/`: Public interface mapping and runtime templates for the Point-LIO, GLIM, and NDT navigation stack used by the SMT workflow. It documents the `/set_nav_goal_2D` navigation service, 2D mapping/navigation startup scripts, relocalization score checks, and a testable dynamic-obstacle halt monitor. Sensitive deployment artifacts are intentionally excluded.

## Notes
- This repository is a research release under active development.
- Trained weights, private datasets, runtime images, production maps, and
  deployment-specific calibration values are intentionally excluded.
- Public documentation should stay at the level of reproducible module
  interfaces and local calibration requirements.

## Contact
For further information or inquiries, please contact the authors of the paper.
