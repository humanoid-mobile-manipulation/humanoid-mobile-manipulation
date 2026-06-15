"""Public Point-LIO/GLIM/NDT navigation pipeline reference.

This module records the paper-level navigation interfaces without exposing
private launch files, host credentials, Docker image links, or factory-specific
calibration values.
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class NavigationInterface:
    name: str
    kind: str
    path_or_topic: str
    purpose: str


NAVIGATION_PIPELINE_INTERFACES: Tuple[NavigationInterface, ...] = (
    NavigationInterface(
        name="MID360 LiDAR input",
        kind="source",
        path_or_topic="navigation/point_lio_glim_ndt/scripts/start_mid360_driver.sh",
        purpose="Starts the MID360 ROS driver and publishes LiDAR and IMU data.",
    ),
    NavigationInterface(
        name="Point-LIO odometry",
        kind="pipeline_stage",
        path_or_topic="Point-LIO",
        purpose="Estimates high-rate local robot motion from LiDAR and IMU data.",
    ),
    NavigationInterface(
        name="GLIM mapping and localization",
        kind="pipeline_stage",
        path_or_topic="GLIM",
        purpose="Builds or loads the map used for workstation navigation.",
    ),
    NavigationInterface(
        name="NDT scan matching",
        kind="pipeline_stage",
        path_or_topic="NDT localization",
        purpose="Refines the robot pose against the map during navigation.",
    ),
    NavigationInterface(
        name="World-to-base pose",
        kind="ros_topic",
        path_or_topic="/T_w_from_b",
        purpose="Publishes the robot base pose used to calibrate standing targets.",
    ),
    NavigationInterface(
        name="Rack-facing target pose",
        kind="source",
        path_or_topic="manipulation/configs/config_real.py",
        purpose="Stores calibrated SMT standing poses as x, y, and yaw.",
    ),
    NavigationInterface(
        name="Navigation command service",
        kind="ros_service",
        path_or_topic="/set_nav_goal_2D",
        purpose="Receives the selected rack-facing target pose from task logic.",
    ),
    NavigationInterface(
        name="Navigation stop service",
        kind="ros_service",
        path_or_topic="/stop_nav_goal_2D",
        purpose="Cancels the active navigation goal for safety stops or task transitions.",
    ),
    NavigationInterface(
        name="Task-level navigation helper",
        kind="source",
        path_or_topic="manipulation/strategy_sps.py:start_navigation",
        purpose="Sends calibrated target poses to the navigation command service.",
    ),
    NavigationInterface(
        name="Dynamic-obstacle halt",
        kind="source",
        path_or_topic="navigation/point_lio_glim_ndt/dynamic_obstacle_halt_monitor.py",
        purpose="Implements stop-zone detection and publishes a navigation halt flag.",
    ),
    NavigationInterface(
        name="2D navigation startup",
        kind="source",
        path_or_topic="navigation/point_lio_glim_ndt/scripts/start_2d_navigation.sh",
        purpose="Starts move_base or straight command-velocity navigation; straight mode can enable obstacle detection.",
    ),
    NavigationInterface(
        name="2D mapping startup",
        kind="source",
        path_or_topic="navigation/point_lio_glim_ndt/scripts/start_2d_mapping.sh",
        purpose="Starts 2D map construction for navigation from the LiDAR SLAM map.",
    ),
    NavigationInterface(
        name="Relocalization score check",
        kind="source",
        path_or_topic="navigation/point_lio_glim_ndt/scripts/check_relocalization_score.sh",
        purpose="Reads the latest NDT relocalization score and compares it with the configured threshold.",
    ),
)


def describe_navigation_pipeline() -> str:
    """Return a human-readable summary of public navigation interfaces."""
    lines = ["Point-LIO/GLIM/NDT public navigation interfaces:"]
    for item in NAVIGATION_PIPELINE_INTERFACES:
        lines.append(f"- {item.name} [{item.kind}]: {item.path_or_topic} -- {item.purpose}")
    return "\n".join(lines)


if __name__ == "__main__":
    print(describe_navigation_pipeline())
