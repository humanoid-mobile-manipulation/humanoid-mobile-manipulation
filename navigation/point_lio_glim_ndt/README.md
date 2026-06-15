# Point-LIO, GLIM, and NDT Navigation Pipeline

This directory documents the public navigation reference for the paper-specific
SMT mobile-manipulation workflow. It is intentionally sanitized: private Docker
images, internal download links, host credentials, and factory-specific pose
values are not included.

## Paper Mapping

The manuscript uses a LiDAR-inertial navigation stack to move the humanoid robot
to a calibrated rack-facing standing pose before AR-tag localization and KMPC
manipulation.

| Paper component | Public reference | Role |
| --- | --- | --- |
| LiDAR-inertial odometry | Point-LIO stage | Produces high-rate local motion estimation from MID360 LiDAR and IMU data. |
| Mapping/localization | GLIM stage | Builds or loads the environment map used by the SMT workstation workflow. |
| Scan matching | NDT localization stage | Refines map-based pose estimates during navigation. |
| Robot base pose output | `/T_w_from_b` | Publishes the world-to-base pose used to record standing poses. |
| Navigation command input | `/set_nav_goal_2D` | Receives calibrated `(x, y, yaw)` targets for rack-facing navigation. |
| Navigation stop input | `/stop_nav_goal_2D` | Cancels the active navigation goal when a safety stop or task transition requires it. |
| Command-velocity bridge | `scripts/start_2d_navigation.sh` with `NAV_TYPE=straight` | Converts navigation output into lower-body velocity commands. |
| Dynamic-obstacle halt | `scripts/start_2d_navigation.sh` with `USE_OBSTACLE_DETECTION=true`, plus `dynamic_obstacle_halt_monitor.py` | Pauses navigation when an obstacle violates the configured safety region. |

## Public Execution Outline

1. Start the MID360 LiDAR driver and time synchronization stack.
2. Start the Point-LIO/GLIM/NDT localization stack with the selected map.
3. Verify that the robot base pose is published on `/T_w_from_b`.
4. Drive the robot to the workstation and record the rack-facing standing pose
   as `(x, y, yaw)`.
5. Store the calibrated standing pose in the manipulation configuration used by
   the task-level strategy.
6. During SMT execution, send the selected standing pose through `/set_nav_goal_2D`.
7. The dynamic-obstacle halt interface should stop or pause navigation before
   the mobile manipulation stage is allowed to continue.

The exact runtime image, map files, network addresses, and calibrated poses
depend on the robot computer, LiDAR driver, container layout, and deployment
map. Those site-specific artifacts are omitted because they are not required to
understand the paper algorithm.

## Documentation Detail Boundary

The original deployment used machine-specific shell scripts, runtime containers,
and operator procedures. The public release keeps only the algorithmic and
interface-level steps:

- how the LiDAR driver, SLAM/localization stack, 2D navigation, and velocity
  bridge are ordered;
- which public topics and services connect navigation to manipulation;
- where local calibrations such as LiDAR extrinsics, NDT thresholds, maps, and
  rack-facing stand poses must be supplied;
- how obstacle halt logic is configured and tested.

Private image names, cloud download links, operator credentials, fixed
workstation paths, and recorded production maps should remain outside the
public repository.

## Runtime Templates

The scripts in `scripts/` are sanitized public templates derived from the
navigation deployment flow used by the project. They preserve the runtime
sequence while replacing private names, image tags, paths, network addresses,
and site-specific poses with local configuration variables.

| Path | Purpose |
| --- | --- |
| `config/navigation_runtime.template.sh` | Shared runtime configuration for container name, runtime paths, SLAM mode, navigation mode, and obstacle-detection switches. |
| `config/mid360_sensor_config.template.json` | MID360 network and extrinsic calibration template with placeholder IP values. |
| `scripts/start_mid360_driver.sh` | Starts the MID360 ROS driver on the host or inside the runtime container. |
| `scripts/start_lidar_slam.sh` | Starts the LiDAR SLAM/localization stack with the selected Point-LIO/GLIM/NDT configuration. |
| `scripts/start_2d_mapping.sh` | Starts 2D navigation map construction from the LiDAR SLAM map. |
| `scripts/generate_2d_navigation_map.sh` | Exports the generated 2D navigation map. |
| `scripts/start_2d_navigation.sh` | Starts either `move_base` navigation or straight command-velocity navigation. Straight mode can enable obstacle detection and halt. |
| `scripts/stop_navigation_task.sh` | Stops the active navigation task through the public stop interface. |
| `scripts/check_relocalization_score.sh` | Reads the latest relocalization result and compares it with the configured NDT alignment threshold. |

## Interfaces

Use `navigation_pipeline_interfaces.py` for the public topic/source mapping. Use
`dynamic_obstacle_halt_monitor.py` for the released stop-zone decision logic and
optional ROS point-cloud adapter.

## Reproduction Notes

To reproduce the navigation component in a new lab, provide:

- LiDAR-to-base and IMU-to-base extrinsic calibration;
- map data generated by the selected Point-LIO/GLIM workflow;
- NDT localization parameters for the target workspace;
- rack-facing standing poses as `(x, y, yaw)` in radians;
- obstacle stop region, stop timeout, and resume policy for the deployment.

The recorded standing pose can be obtained by positioning the robot at the
desired rack-facing grasp location, reading `/T_w_from_b`, and storing the
resulting `(x, y, yaw)` in the task-level manipulation configuration. The yaw
value should be stored in radians.

For double-blind release, do not publish local runtime images, private download
links, organization-specific path names, host credentials, fixed lab IPs, or
recorded production maps.
