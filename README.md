# Humanoid Mobile Manipulation Repository

This repository is part of our ongoing research work, which is currently under submission. It contains the implementation of various components for humanoid mobile manipulation tasks. Below is an overview of the repository structure and its purpose:

## Repository Structure

### Module Overview

| Module | Repository location | Notes |
| --- | --- | --- |
| SPS YOLOv12/RandLA-Net perception pipeline | `perception/sps_yolo_randla/` | RGB-D capture, data preparation, YOLO segmentation training, candidate-mask score labeling, RandLA-Net scorer training, and whole-image inference reference. |
| AR-tag/KMPC manipulation reference | `manipulation/smt_ar_tag_kmpc/` | Reference for AR-tag localization, light-selected target IDs, calibrated navigation stand poses, and model-based KMPC/mobile manipulation integration. |
| Shared Point-LIO/GLIM/NDT navigation pipeline | `navigation/point_lio_glim_ndt/` | Runtime templates for LiDAR-inertial localization, NDT scan matching, 2D mapping/navigation, navigation pose calibration, and dynamic-obstacle halt logic used by the task workflows. |
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

## Reproduction Overview

The reproduction path is organized by subsystem. Each module README lists the
main scripts, ROS interfaces, and local calibration inputs needed to run that
part of the workflow.

1. Initialize the repository submodules so that the Kuavo control tree and the
   imitation-learning framework are available.
2. Configure the shared navigation stack in `navigation/point_lio_glim_ndt/` to
   build or load the map, record rack-facing stand poses, run 2D navigation,
   and enable dynamic-obstacle halt logic.
3. For the SPS workflow, use `perception/sps_yolo_randla/` for YOLO/RandLA-Net
   candidate selection and `imitation_learning_framework/IL/` for the
   learning-based manipulation policy.
4. For the SMT workflow, use `manipulation/smt_ar_tag_kmpc/` for AR-tag
   localization, selected target IDs, stand-pose lookup, and model-based
   KMPC/Bezier manipulation interfaces.
5. Use `kuavo_ros_application/kuavo/` as the shared robot-control dependency
   for low-level control, messages, services, KMPC, Bezier planning, and
   single-step refinement.

The README files in each module are concise public guides. They document what
to run, which topics/services connect the modules, and which local calibration
values a new deployment must provide.

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
- `smt_ar_tag_kmpc/`: AR-tag/KMPC pipeline interface reference.

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
- `point_lio_glim_ndt/`: Interface mapping and runtime templates for the Point-LIO, GLIM, and NDT navigation stack shared by the SPS and SMT workflows. It documents the `/set_nav_goal_2D` navigation service, 2D mapping/navigation startup scripts, relocalization score checks, and a testable dynamic-obstacle halt monitor.

## Notes
- This repository is a research release under active development.
- Trained weights, datasets, maps, and deployment-specific calibration values
  should be supplied for each local setup.

## Contact
For further information or inquiries, please contact the authors of the paper.
