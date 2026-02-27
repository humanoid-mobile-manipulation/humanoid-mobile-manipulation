# Humanoid Mobile Manipulation Repository

This repository is part of our ongoing research work, which is currently under submission. It contains the implementation of various components for humanoid mobile manipulation tasks. Below is an overview of the repository structure and its purpose:

## Repository Structure

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

## Notes
- This repository is a work in progress, and some components are still under development.
- If you require specific implementation details or have any questions, feel free to contact us.

## Contact
For further information or inquiries, please contact the authors of the paper.

---

Thank you for your interest in our work!