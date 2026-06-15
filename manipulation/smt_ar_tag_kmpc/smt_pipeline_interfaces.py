"""Public SMT AR-tag/KMPC pipeline interface reference.

This file intentionally contains no private launch commands, credentials,
factory-specific coordinates, or internal artifact links. It records the public
interfaces and repository paths that connect the SMT pipeline described in the
paper.
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class PipelineInterface:
    name: str
    kind: str
    path_or_topic: str
    purpose: str


SMT_PIPELINE_INTERFACES: Tuple[PipelineInterface, ...] = (
    PipelineInterface(
        name="AR tag detections",
        kind="ros_topic",
        path_or_topic="/robot_tag_info",
        purpose="Publishes detected AR-tag IDs and poses for the rack or tray.",
    ),
    PipelineInterface(
        name="Red-light target IDs",
        kind="ros_topic",
        path_or_topic="/red_tag_ids",
        purpose="Publishes the target tag IDs selected by the active red indicator light.",
    ),
    PipelineInterface(
        name="Blue-light target IDs",
        kind="ros_topic",
        path_or_topic="/blue_tag_ids",
        purpose="Publishes the target tag IDs selected by the active blue indicator light.",
    ),
    PipelineInterface(
        name="Navigation command service",
        kind="ros_service",
        path_or_topic="/set_nav_goal_2D",
        purpose="Receives calibrated standing poses for navigation to the SMT rack.",
    ),
    PipelineInterface(
        name="Navigation stop service",
        kind="ros_service",
        path_or_topic="/stop_nav_goal_2D",
        purpose="Cancels the active navigation goal during safety stops or task transitions.",
    ),
    PipelineInterface(
        name="SMT tag helper",
        kind="source",
        path_or_topic="manipulation/strategy_sps.py:subscribe_and_print_tag_info",
        purpose="Reads the selected AR-tag pose from the detection topic.",
    ),
    PipelineInterface(
        name="SMT target-ID helper",
        kind="source",
        path_or_topic="manipulation/strategy_sps.py:wait_tag_ids",
        purpose="Blocks until the active-light target ID list is available.",
    ),
    PipelineInterface(
        name="Navigation helper",
        kind="source",
        path_or_topic="manipulation/strategy_sps.py:start_navigation",
        purpose="Sends the robot to the calibrated rack-facing stand pose.",
    ),
    PipelineInterface(
        name="Navigation pose config",
        kind="source",
        path_or_topic="manipulation/configs/config_real.py",
        purpose="Stores public examples of stand-pose and offset configuration fields.",
    ),
    PipelineInterface(
        name="KMPC controller",
        kind="source",
        path_or_topic="kuavo_ros_application/kuavo/src/humanoid-control/mobile_manipulator_controllers/",
        purpose="Contains the model-based mobile manipulation controller implementation.",
    ),
)


def describe_pipeline() -> str:
    """Return a human-readable summary of public SMT pipeline interfaces."""
    lines = ["SMT AR-tag/KMPC public pipeline interfaces:"]
    for item in SMT_PIPELINE_INTERFACES:
        lines.append(f"- {item.name} [{item.kind}]: {item.path_or_topic} -- {item.purpose}")
    return "\n".join(lines)


if __name__ == "__main__":
    print(describe_pipeline())
