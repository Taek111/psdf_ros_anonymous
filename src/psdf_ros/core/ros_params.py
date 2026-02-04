"""ROS parameter loading for PSDF-ROS nodes."""

from dataclasses import dataclass, field
from typing import List

import rospy
import yaml


@dataclass
class PsdfRosParams:
    """Container for PSDF-ROS parameters loaded from the ROS parameter server."""

    horizon: int = 15
    dt: float = 0.1
    ref_ds: float = 0.0
    obstacle_topic: str = "/obstacles"
    line_segment_topic: str = "/line_segments"
    robot_base: str = "base_link"
    global_frame: str = "odom"
    d_safe: float = 0.2
    max_clusters: int = 20
    max_edges_per_cluster: int = 64
    emergency_stop_on_fail: bool = True
    optimizer_config_file: str = ""
    vehicle_model: str = "differential"
    vmin: float = -1.0
    vmax: float = 1.0
    omegamin: float = -1.5
    omegamax: float = 1.5
    wheelbase: float = 1.0
    steering_min: float = -0.6
    steering_max: float = 0.6
    initial_steering: float = 0.0
    Q: List[float] = field(default_factory=lambda: [50.0, 50.0, 1.0])
    R: List[float] = field(default_factory=lambda: [0.2, 0.05])
    terminal_weight: float = 2.0
    approach_slowdown: bool = True
    slowdown_radius: float = 2.0
    min_v_scale: float = 0.2
    min_omega_scale: float = 0.3
    v_sign_eps: float = 0.2
    v_sign_min_interval: float = 1.0
    footprint: List[List[float]] = field(default_factory=lambda: [
        [-0.25, -0.25],
        [0.25, -0.25],
        [0.25, 0.25],
        [-0.25, 0.25],
    ])

    @classmethod
    def from_ros(cls, ns: str = "~") -> "PsdfRosParams":
        """Load parameters from the ROS parameter server."""

        def gp(name: str, default):
            return rospy.get_param(f"{ns}{name}", default)

        params = cls(
            horizon=gp("horizon", cls.horizon),
            dt=gp("dt", cls.dt),
            ref_ds=gp("ref_ds", cls.ref_ds),
            obstacle_topic=gp("obstacle_topic", cls.obstacle_topic),
            line_segment_topic=gp("line_segment_topic", cls.line_segment_topic),
            robot_base=gp("robot_base", cls.robot_base),
            global_frame=gp("global_frame", cls.global_frame),
            d_safe=gp("d_safe", cls.d_safe),
            max_clusters=gp("max_clusters", cls.max_clusters),
            max_edges_per_cluster=gp("max_edges_per_cluster", cls.max_edges_per_cluster),
            emergency_stop_on_fail=gp("emergency_stop_on_fail", cls.emergency_stop_on_fail),
            optimizer_config_file=gp("optimizer_config_file", cls.optimizer_config_file),
            vehicle_model=str(gp("vehicle_model", cls.vehicle_model)).lower(),
            vmin=gp("vmin", cls.vmin),
            vmax=gp("vmax", cls.vmax),
            omegamin=gp("omegamin", cls.omegamin),
            omegamax=gp("omegamax", cls.omegamax),
            wheelbase=gp("wheelbase", cls.wheelbase),
            steering_min=gp("steering_min", cls.steering_min),
            steering_max=gp("steering_max", cls.steering_max),
            initial_steering=gp("initial_steering", cls.initial_steering),
            Q=gp("Q", cls.Q),
            R=gp("R", cls.R),
            terminal_weight=gp("terminal_weight", cls.terminal_weight),
            approach_slowdown=gp("approach_slowdown", cls.approach_slowdown),
            slowdown_radius=gp("slowdown_radius", cls.slowdown_radius),
            min_v_scale=gp("min_v_scale", cls.min_v_scale),
            min_omega_scale=gp("min_omega_scale", cls.min_omega_scale),
            v_sign_eps=gp("v_sign_eps", cls.v_sign_eps),
            v_sign_min_interval=gp("v_sign_min_interval", cls.v_sign_min_interval),
        )

        # Backward-compat: accept '~wheel_base' as synonym
        try:
            wb_yaml = rospy.get_param(f"{ns}wheel_base")
            params.wheelbase = float(wb_yaml)
        except Exception:
            pass

        footprint_file = gp("footprint", "")
        if footprint_file:
            try:
                with open(footprint_file, "r") as f:
                    fp_yaml = yaml.safe_load(f)
                params.footprint = fp_yaml.get("footprint", [])
                rospy.loginfo_throttle(1.0, f"Loaded footprint: {params.footprint}")
            except Exception as exc:
                rospy.logwarn(f"Failed to load footprint file: {exc}")

        return params
