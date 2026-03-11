#!/usr/bin/env python3

"""Live plots for per-leg average actuator torque and speed."""

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, List, Optional, Sequence, Tuple

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
MPL_CONFIG_DIR = Path("/tmp/quad_ocs2_matplotlib")
MPL_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CONFIG_DIR))

try:
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover - environment-specific
    raise SystemExit(
        "matplotlib is required for plotting. Install it first, then rerun this script."
    ) from exc

try:
    import rclpy
    from rclpy.node import Node

    from legged_msgs.msg import JointControlData, SimulatorStateData
except ImportError as exc:  # pragma: no cover - environment-specific
    raise SystemExit(
        "ROS 2 Python packages were not found. Source your workspace first:\n"
        "  source /home/kaan/quad_ocs2_ws/install/setup.bash"
    ) from exc


RPM_PER_RAD_PER_SEC = 60.0 / (2.0 * math.pi)
DEFAULT_JOINT_ORDER = [
    "LF_HAA",
    "LF_HFE",
    "LF_KFE",
    "LH_HAA",
    "LH_HFE",
    "LH_KFE",
    "RF_HAA",
    "RF_HFE",
    "RF_KFE",
    "RH_HAA",
    "RH_HFE",
    "RH_KFE",
]
DEFAULT_ROBOT_XML = WORKSPACE_ROOT / (
    "src/Quadruped-Control-OCS2-ROS2/legged_control/"
    "mujoco_simulator/models/quad_mini/urdf/robot.xml"
)
DEFAULT_SIM_CONFIG = WORKSPACE_ROOT / (
    "src/Quadruped-Control-OCS2-ROS2/legged_control/"
    "user_command/config/quad_mini/simulation.info"
)
FLOAT_PATTERN = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"


@dataclass
class ControlSnapshot:
    joint_torque: List[float]
    joint_position: List[float]
    joint_velocity: List[float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot per-leg average actuator torque and joint speed without changing "
            "the simulator or controller."
        )
    )
    parser.add_argument(
        "--state-topic",
        default="simulator_state_data",
        help="Simulator state topic carrying measured joint position and velocity.",
    )
    parser.add_argument(
        "--control-topic",
        default="joint_control_data",
        help="Controller command topic carrying feedforward torque and desired joints.",
    )
    parser.add_argument(
        "--robot-xml",
        type=Path,
        default=DEFAULT_ROBOT_XML if DEFAULT_ROBOT_XML.exists() else None,
        help=(
            "MuJoCo robot XML used to detect joint ordering. Defaults to quad_mini if present. "
            "Pass your own XML when using another robot."
        ),
    )
    parser.add_argument(
        "--sim-config",
        type=Path,
        default=None,
        help=(
            "Simulation config containing pid.kp and pid.kd. If omitted, the script tries to infer "
            "it from --robot-xml and then falls back to the quad_mini config."
        ),
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=None,
        help="Override the proportional gain used by the simulator torque law.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=None,
        help="Override the derivative gain used by the simulator torque law.",
    )
    parser.add_argument(
        "--window-seconds",
        type=float,
        default=30.0,
        help="How much history to keep visible in the live plots.",
    )
    parser.add_argument(
        "--refresh-hz",
        type=float,
        default=10.0,
        help="Plot refresh rate.",
    )
    parser.add_argument(
        "--signed",
        action="store_true",
        help="Use signed averages instead of average magnitudes.",
    )
    return parser.parse_args()


def infer_sim_config_from_robot_xml(robot_xml: Path) -> Optional[Path]:
    try:
        if robot_xml.parents[3].name != "mujoco_simulator" or robot_xml.parents[2].name != "models":
            return None
        robot_type = robot_xml.parent.parent.name
        legged_control_root = robot_xml.parents[4]
    except IndexError:
        return None

    sim_config = legged_control_root / "user_command" / "config" / robot_type / "simulation.info"
    return sim_config if sim_config.exists() else None


def extract_joint_order(robot_xml: Optional[Path]) -> List[str]:
    if robot_xml is None:
        return DEFAULT_JOINT_ORDER.copy()

    robot_xml = robot_xml.expanduser().resolve()
    if not robot_xml.exists():
        raise FileNotFoundError(f"Robot XML not found: {robot_xml}")

    root = ET.parse(robot_xml).getroot()
    sensor = root.find("sensor")
    if sensor is None:
        raise ValueError(f"No <sensor> block found in {robot_xml}")

    for tag_name in ("jointpos", "jointvel"):
        joints = [element.attrib["joint"] for element in sensor.findall(tag_name) if "joint" in element.attrib]
        if len(joints) >= 12:
            return joints[:12]

    raise ValueError(f"Could not determine 12 joint names from {robot_xml}")


def build_leg_map(joint_order: Sequence[str]) -> Tuple[List[str], Dict[str, List[int]]]:
    leg_order: List[str] = []
    leg_map: Dict[str, List[int]] = {}
    for index, joint_name in enumerate(joint_order):
        leg_name = joint_name.split("_", 1)[0]
        if leg_name not in leg_map:
            leg_map[leg_name] = []
            leg_order.append(leg_name)
        leg_map[leg_name].append(index)

    return leg_order, leg_map


def load_pid_gains(sim_config: Optional[Path], kp_override: Optional[float], kd_override: Optional[float]) -> Tuple[float, float, Optional[Path]]:
    if kp_override is not None and kd_override is not None:
        return kp_override, kd_override, sim_config

    resolved_config = sim_config
    if resolved_config is None and DEFAULT_SIM_CONFIG.exists():
        resolved_config = DEFAULT_SIM_CONFIG

    if resolved_config is None:
        kp = 0.0 if kp_override is None else kp_override
        kd = 0.0 if kd_override is None else kd_override
        return kp, kd, None

    resolved_config = resolved_config.expanduser().resolve()
    if not resolved_config.exists():
        raise FileNotFoundError(f"Simulation config not found: {resolved_config}")

    text = resolved_config.read_text(encoding="utf-8")
    pid_block = re.search(r"pid\s*\{(?P<body>.*?)\}", text, flags=re.DOTALL)
    search_text = pid_block.group("body") if pid_block else text

    kp = kp_override if kp_override is not None else extract_scalar(search_text, "kp")
    kd = kd_override if kd_override is not None else extract_scalar(search_text, "kd")
    return kp, kd, resolved_config


def extract_scalar(text: str, key: str) -> float:
    match = re.search(rf"\b{re.escape(key)}\b\s+({FLOAT_PATTERN})", text)
    if not match:
        raise ValueError(f"Could not find '{key}' in simulation config.")
    return float(match.group(1))


def average(values: Sequence[float]) -> float:
    return sum(values) / float(len(values))


class LegMetricPlotter(Node):
    def __init__(
        self,
        state_topic: str,
        control_topic: str,
        leg_order: Sequence[str],
        leg_map: Dict[str, List[int]],
        kp: float,
        kd: float,
        window_seconds: float,
        refresh_hz: float,
        signed: bool,
    ) -> None:
        super().__init__("leg_metric_plotter")
        self.leg_order = list(leg_order)
        self.leg_map = leg_map
        self.kp = kp
        self.kd = kd
        self.window_seconds = max(window_seconds, 1.0)
        self.signed = signed
        self.latest_command: Optional[ControlSnapshot] = None
        self.times: Deque[float] = deque()
        self.torque_history: Dict[str, Deque[float]] = {leg: deque() for leg in self.leg_order}
        self.speed_history: Dict[str, Deque[float]] = {leg: deque() for leg in self.leg_order}

        self._setup_plots()

        self.create_subscription(JointControlData, control_topic, self.control_callback, 10)
        self.create_subscription(SimulatorStateData, state_topic, self.state_callback, 10)
        self.create_timer(1.0 / max(refresh_hz, 1.0), self.refresh_plots)

        self.get_logger().info(
            f"Listening on '{state_topic}' and '{control_topic}'. "
            f"Using kp={self.kp:.3f}, kd={self.kd:.3f}, legs={', '.join(self.leg_order)}."
        )

    def control_callback(self, msg: JointControlData) -> None:
        expected = max(index for indices in self.leg_map.values() for index in indices) + 1
        if len(msg.joint_torque) < expected or len(msg.joint_position) < expected or len(msg.joint_velocity) < expected:
            self.get_logger().warning("Ignoring control sample with incomplete joint arrays.")
            return

        self.latest_command = ControlSnapshot(
            joint_torque=list(msg.joint_torque[:expected]),
            joint_position=list(msg.joint_position[:expected]),
            joint_velocity=list(msg.joint_velocity[:expected]),
        )

    def state_callback(self, msg: SimulatorStateData) -> None:
        if self.latest_command is None:
            return

        expected = max(index for indices in self.leg_map.values() for index in indices) + 1
        if len(msg.joint_position_values) < expected or len(msg.joint_velocity_values) < expected:
            self.get_logger().warning("Ignoring state sample with incomplete joint arrays.")
            return

        sim_time = float(msg.simulation_time)
        if self.times and sim_time < self.times[-1]:
            self.get_logger().info("Simulation time moved backwards. Clearing history.")
            self.clear_history()

        applied_torque: List[float] = []
        for index in range(expected):
            torque = self.latest_command.joint_torque[index]
            torque += self.kp * (self.latest_command.joint_position[index] - msg.joint_position_values[index])
            torque += self.kd * (self.latest_command.joint_velocity[index] - msg.joint_velocity_values[index])
            applied_torque.append(torque)

        self.times.append(sim_time)
        for leg_name in self.leg_order:
            joint_indices = self.leg_map[leg_name]
            leg_torque = [applied_torque[index] for index in joint_indices]
            leg_speed = [msg.joint_velocity_values[index] * RPM_PER_RAD_PER_SEC for index in joint_indices]

            if not self.signed:
                leg_torque = [abs(value) for value in leg_torque]
                leg_speed = [abs(value) for value in leg_speed]

            self.torque_history[leg_name].append(average(leg_torque))
            self.speed_history[leg_name].append(average(leg_speed))

        self.trim_history()

    def clear_history(self) -> None:
        self.times.clear()
        for leg_name in self.leg_order:
            self.torque_history[leg_name].clear()
            self.speed_history[leg_name].clear()

    def trim_history(self) -> None:
        while self.times and (self.times[-1] - self.times[0]) > self.window_seconds:
            self.times.popleft()
            for leg_name in self.leg_order:
                self.torque_history[leg_name].popleft()
                self.speed_history[leg_name].popleft()

    def _setup_plots(self) -> None:
        plt.ion()
        rows = math.ceil(len(self.leg_order) / 2.0)
        cols = 2 if len(self.leg_order) > 1 else 1

        self.torque_fig, torque_axes = plt.subplots(rows, cols, squeeze=False, figsize=(12, 6.5), num="Average Leg Torque")
        self.speed_fig, speed_axes = plt.subplots(rows, cols, squeeze=False, figsize=(12, 6.5), num="Average Leg Speed")

        self.torque_axes = torque_axes.flatten()
        self.speed_axes = speed_axes.flatten()
        self.torque_lines = {}
        self.speed_lines = {}

        torque_ylabel = "Average torque [Nm]" if self.signed else "Average |torque| [Nm]"
        speed_ylabel = "Average speed [RPM]" if self.signed else "Average |speed| [RPM]"

        for index, leg_name in enumerate(self.leg_order):
            torque_axis = self.torque_axes[index]
            speed_axis = self.speed_axes[index]

            self.torque_lines[leg_name], = torque_axis.plot([], [], linewidth=2.0, color="#c2410c")
            self.speed_lines[leg_name], = speed_axis.plot([], [], linewidth=2.0, color="#0369a1")

            torque_axis.set_title(leg_name)
            speed_axis.set_title(leg_name)
            torque_axis.set_ylabel(torque_ylabel)
            speed_axis.set_ylabel(speed_ylabel)
            torque_axis.grid(True, alpha=0.35)
            speed_axis.grid(True, alpha=0.35)

        for index in range(len(self.leg_order), len(self.torque_axes)):
            self.torque_axes[index].set_visible(False)
            self.speed_axes[index].set_visible(False)

        for axis in self.torque_axes[: len(self.leg_order)]:
            axis.set_xlabel("Simulation time [s]")
        for axis in self.speed_axes[: len(self.leg_order)]:
            axis.set_xlabel("Simulation time [s]")

        self.torque_fig.tight_layout()
        self.speed_fig.tight_layout()
        plt.show(block=False)

    def refresh_plots(self) -> None:
        if not plt.fignum_exists(self.torque_fig.number) or not plt.fignum_exists(self.speed_fig.number):
            self.get_logger().info("Plot window closed. Shutting down.")
            rclpy.shutdown()
            return

        if not self.times:
            plt.pause(0.001)
            return

        x_values = list(self.times)
        x_min = x_values[0]
        x_max = x_values[-1] if x_values[-1] > x_min else x_min + 1e-3

        for leg_name in self.leg_order:
            torque_values = list(self.torque_history[leg_name])
            speed_values = list(self.speed_history[leg_name])

            self.torque_lines[leg_name].set_data(x_values, torque_values)
            self.speed_lines[leg_name].set_data(x_values, speed_values)

            torque_axis = self._axis_for_leg(self.torque_axes, leg_name)
            speed_axis = self._axis_for_leg(self.speed_axes, leg_name)

            torque_axis.set_xlim(x_min, x_max)
            speed_axis.set_xlim(x_min, x_max)
            self._set_y_limits(torque_axis, torque_values)
            self._set_y_limits(speed_axis, speed_values)

            torque_axis.set_title(f"{leg_name}  latest: {torque_values[-1]:.2f} Nm")
            speed_axis.set_title(f"{leg_name}  latest: {speed_values[-1]:.2f} RPM")

        self.torque_fig.canvas.draw_idle()
        self.speed_fig.canvas.draw_idle()
        plt.pause(0.001)

    def _axis_for_leg(self, axes: Sequence[plt.Axes], leg_name: str):
        return axes[self.leg_order.index(leg_name)]

    def _set_y_limits(self, axis, values: Sequence[float]) -> None:
        if not values:
            return

        value_min = min(values)
        value_max = max(values)

        if self.signed:
            if math.isclose(value_min, value_max):
                pad = max(0.5, abs(value_max) * 0.1)
                axis.set_ylim(value_min - pad, value_max + pad)
            else:
                pad = (value_max - value_min) * 0.1
                axis.set_ylim(value_min - pad, value_max + pad)
            return

        upper = value_max if value_max > 0.0 else 1.0
        axis.set_ylim(0.0, upper * 1.1)


def main() -> int:
    args = parse_args()

    sim_config = args.sim_config
    if sim_config is None and args.robot_xml is not None:
        inferred = infer_sim_config_from_robot_xml(args.robot_xml)
        if inferred is not None:
            sim_config = inferred

    try:
        joint_order = extract_joint_order(args.robot_xml)
        leg_order, leg_map = build_leg_map(joint_order)
        kp, kd, resolved_sim_config = load_pid_gains(sim_config, args.kp, args.kd)
    except (FileNotFoundError, ValueError, ET.ParseError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    if resolved_sim_config is not None:
        print(f"Using simulation gains from: {resolved_sim_config}")
    elif args.kp is None or args.kd is None:
        print("Warning: no simulation config found, using zero PID gains.", file=sys.stderr)

    if args.robot_xml is not None:
        print(f"Detected joint order from: {args.robot_xml}")
    else:
        print("Using built-in joint order fallback.")

    print(f"Leg grouping: {', '.join(f'{leg}={leg_map[leg]}' for leg in leg_order)}")

    rclpy.init()
    node = LegMetricPlotter(
        state_topic=args.state_topic,
        control_topic=args.control_topic,
        leg_order=leg_order,
        leg_map=leg_map,
        kp=kp,
        kd=kd,
        window_seconds=args.window_seconds,
        refresh_hz=args.refresh_hz,
        signed=args.signed,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.close("all")
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
