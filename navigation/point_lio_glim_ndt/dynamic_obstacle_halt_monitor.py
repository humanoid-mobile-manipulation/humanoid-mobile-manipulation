"""Dynamic-obstacle halt logic for navigation safety.

The core logic is ROS-independent so it can be tested without robot hardware.
The optional ROS entry point subscribes to a point cloud, evaluates whether
points enter a configured stop zone in the robot base frame, and publishes a
halt flag for the navigation supervisor.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple


Point3D = Tuple[float, float, float]


@dataclass(frozen=True)
class ObstacleHaltConfig:
    """Stop-zone configuration in the robot base frame.

    The coordinate convention is x forward, y left, and z upward.
    """

    forward_min_m: float = 0.0
    forward_max_m: float = 1.2
    lateral_half_width_m: float = 0.45
    min_height_m: float = -0.2
    max_height_m: float = 1.2
    min_points_to_halt: int = 3
    clear_frames_to_resume: int = 3


@dataclass(frozen=True)
class HaltDecision:
    should_halt: bool
    obstacle_points: int
    clear_frame_count: int
    reason: str


class DynamicObstacleHaltMonitor:
    """Stateful stop-zone monitor with hysteresis for resume decisions."""

    def __init__(self, config: Optional[ObstacleHaltConfig] = None):
        self.config = config or ObstacleHaltConfig()
        self._halted = False
        self._clear_frame_count = 0

    @property
    def halted(self) -> bool:
        return self._halted

    def count_points_in_stop_zone(self, points: Iterable[Sequence[float]]) -> int:
        count = 0
        cfg = self.config

        for point in points:
            if len(point) < 3:
                continue

            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            in_forward_range = cfg.forward_min_m <= x <= cfg.forward_max_m
            in_lateral_range = abs(y) <= cfg.lateral_half_width_m
            in_height_range = cfg.min_height_m <= z <= cfg.max_height_m

            if in_forward_range and in_lateral_range and in_height_range:
                count += 1

        return count

    def update(self, points: Iterable[Sequence[float]]) -> HaltDecision:
        obstacle_points = self.count_points_in_stop_zone(points)
        has_obstacle = obstacle_points >= self.config.min_points_to_halt

        if has_obstacle:
            self._halted = True
            self._clear_frame_count = 0
            return HaltDecision(
                should_halt=True,
                obstacle_points=obstacle_points,
                clear_frame_count=self._clear_frame_count,
                reason="obstacle_in_stop_zone",
            )

        if self._halted:
            self._clear_frame_count += 1
            if self._clear_frame_count >= self.config.clear_frames_to_resume:
                self._halted = False
                return HaltDecision(
                    should_halt=False,
                    obstacle_points=obstacle_points,
                    clear_frame_count=self._clear_frame_count,
                    reason="stop_zone_clear",
                )

            return HaltDecision(
                should_halt=True,
                obstacle_points=obstacle_points,
                clear_frame_count=self._clear_frame_count,
                reason="waiting_for_clear_hysteresis",
            )

        return HaltDecision(
            should_halt=False,
            obstacle_points=obstacle_points,
            clear_frame_count=self._clear_frame_count,
            reason="stop_zone_clear",
        )


def clamp_velocity_when_halted(
    decision: HaltDecision,
    linear_x: float,
    angular_z: float,
) -> Tuple[float, float]:
    """Return a zero command while halted, otherwise pass through velocity."""

    if decision.should_halt:
        return 0.0, 0.0
    return linear_x, angular_z


def run_ros_node() -> None:
    """Run the ROS adapter for the halt monitor.

    Expected integration:
    - input point cloud: obstacle points transformed into the robot base frame;
    - output halt flag: consumed by the navigation supervisor or velocity gate.
    """

    import rospy
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs.point_cloud2 as point_cloud2
    from std_msgs.msg import Bool

    rospy.init_node("dynamic_obstacle_halt_monitor")

    config = ObstacleHaltConfig(
        forward_min_m=rospy.get_param("~forward_min_m", 0.0),
        forward_max_m=rospy.get_param("~forward_max_m", 1.2),
        lateral_half_width_m=rospy.get_param("~lateral_half_width_m", 0.45),
        min_height_m=rospy.get_param("~min_height_m", -0.2),
        max_height_m=rospy.get_param("~max_height_m", 1.2),
        min_points_to_halt=rospy.get_param("~min_points_to_halt", 3),
        clear_frames_to_resume=rospy.get_param("~clear_frames_to_resume", 3),
    )
    monitor = DynamicObstacleHaltMonitor(config)
    halt_pub = rospy.Publisher("~halt", Bool, queue_size=1, latch=True)

    def on_cloud(message: PointCloud2) -> None:
        points = point_cloud2.read_points(
            message,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        decision = monitor.update(points)
        halt_pub.publish(Bool(data=decision.should_halt))
        rospy.logdebug(
            "dynamic obstacle halt=%s obstacle_points=%d reason=%s",
            decision.should_halt,
            decision.obstacle_points,
            decision.reason,
        )

    input_topic = rospy.get_param("~input_cloud_topic", "/navigation/obstacle_points_base")
    rospy.Subscriber(input_topic, PointCloud2, on_cloud, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    run_ros_node()
