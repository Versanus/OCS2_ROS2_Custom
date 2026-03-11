#!/usr/bin/env python3

"""Write per-joint torque and speed samples to a CSV file."""

import argparse
import csv
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

try:
    import rclpy
    from rclpy.node import Node

    from legged_msgs.msg import JointControlData, SimulatorStateData
except ImportError as exc:  # pragma: no cover - environment-specific
    raise SystemExit(
        "ROS 2 Python packages were not found. Source your workspace first:\n"
        "  source /home/kaan/quad_ocs2_ws/install/setup.bash"
    ) from exc


RPM_PER_RAD_PER_SEC = 60.0 / (2.0 * 3.141592653589793)
FLOAT_PATTERN = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"
WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT_XML = WORKSPACE_ROOT / (
    "src/Quadruped-Control-OCS2-ROS2/legged_control/"
    "mujoco_simulator/models/quad_mini/urdf/robot.xml"
)
DEFAULT_SIM_CONFIG = WORKSPACE_ROOT / (
    "src/Quadruped-Control-OCS2-ROS2/legged_control/"
    "user_command/config/quad_mini/simulation.info"
)
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


@dataclass
class ControlSnapshot:
    joint_torque: List[float]
    joint_position: List[float]
    joint_velocity: List[float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "After you start this script, capture the next N seconds of joint torque and "
            "joint speed into two CSV files, then exit automatically."
        )
    )
    parser.add_argument(
        "--state-topic",
        default="simulator_state_data",
        help="Simulator state topic carrying measured joint state.",
    )
    parser.add_argument(
        "--control-topic",
        default="joint_control_data",
        help="Controller topic carrying feedforward torque and desired joints.",
    )
    parser.add_argument(
        "--robot-xml",
        type=Path,
        default=DEFAULT_ROBOT_XML if DEFAULT_ROBOT_XML.exists() else None,
        help=(
            "MuJoCo robot XML used to detect joint ordering. Defaults to quad_mini if present."
        ),
    )
    parser.add_argument(
        "--sim-config",
        type=Path,
        default=None,
        help="Simulation config containing pid.kp and pid.kd.",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=None,
        help="Override simulator proportional gain.",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=None,
        help="Override simulator derivative gain.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Capture duration in simulator seconds.",
    )
    parser.add_argument(
        "--name",
        default="joint",
        help="Capture name used in the default CSV filenames, for example 'stand' or 'trot'.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=WORKSPACE_ROOT,
        help="Directory for the default named CSV files.",
    )
    parser.add_argument(
        "--torque-output",
        type=Path,
        default=None,
        help="Explicit torque CSV output path. Overrides --name for the torque file.",
    )
    parser.add_argument(
        "--speed-output",
        type=Path,
        default=None,
        help="Explicit speed CSV output path. Overrides --name for the speed file.",
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


def load_pid_gains(
    sim_config: Optional[Path], kp_override: Optional[float], kd_override: Optional[float]
) -> Tuple[float, float, Optional[Path]]:
    if kp_override is not None and kd_override is not None:
        return kp_override, kd_override, sim_config

    resolved_config = sim_config
    if resolved_config is None and DEFAULT_SIM_CONFIG.exists():
        resolved_config = DEFAULT_SIM_CONFIG

    if resolved_config is None:
        return (
            0.0 if kp_override is None else kp_override,
            0.0 if kd_override is None else kd_override,
            None,
        )

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


def sanitize_capture_name(name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("._-")
    return sanitized or "joint"


def resolve_output_paths(
    name: str,
    output_dir: Path,
    torque_output: Optional[Path],
    speed_output: Optional[Path],
) -> Tuple[Path, Path]:
    if torque_output is not None and speed_output is not None:
        return torque_output.expanduser().resolve(), speed_output.expanduser().resolve()

    sanitized_name = sanitize_capture_name(name)
    resolved_dir = output_dir.expanduser().resolve()
    default_torque = resolved_dir / f"{sanitized_name}_torque.csv"
    default_speed = resolved_dir / f"{sanitized_name}_speed.csv"

    resolved_torque = torque_output.expanduser().resolve() if torque_output is not None else default_torque
    resolved_speed = speed_output.expanduser().resolve() if speed_output is not None else default_speed
    return resolved_torque, resolved_speed


class JointMetricCsvLogger(Node):
    def __init__(
        self,
        state_topic: str,
        control_topic: str,
        joint_order: Sequence[str],
        kp: float,
        kd: float,
        duration: float,
        torque_output_path: Path,
        speed_output_path: Path,
    ) -> None:
        super().__init__("joint_metric_csv_logger")
        self.joint_order = list(joint_order)
        self.expected_joint_count = len(self.joint_order)
        self.kp = kp
        self.kd = kd
        self.duration = max(duration, 0.001)
        self.latest_command: Optional[ControlSnapshot] = None
        self.capture_start_time: Optional[float] = None
        self.samples_written = 0
        self.finished = False

        self.torque_output_path = torque_output_path.expanduser().resolve()
        self.speed_output_path = speed_output_path.expanduser().resolve()
        self.torque_output_path.parent.mkdir(parents=True, exist_ok=True)
        self.speed_output_path.parent.mkdir(parents=True, exist_ok=True)

        self.torque_output_file = self.torque_output_path.open("w", newline="", encoding="utf-8")
        self.speed_output_file = self.speed_output_path.open("w", newline="", encoding="utf-8")
        self.torque_writer = csv.writer(self.torque_output_file)
        self.speed_writer = csv.writer(self.speed_output_file)
        self.torque_writer.writerow(self._header("torque_nm"))
        self.speed_writer.writerow(self._header("speed_rpm"))
        self.torque_output_file.flush()
        self.speed_output_file.flush()

        self.create_subscription(JointControlData, control_topic, self.control_callback, 10)
        self.create_subscription(SimulatorStateData, state_topic, self.state_callback, 10)

        self.get_logger().info(
            "Ready. Waiting for the first valid simulator sample to start a "
            f"{self.duration:.3f}s capture."
        )

    def _header(self, suffix: str) -> List[str]:
        columns = ["elapsed_time_sec", "simulation_time_sec"]
        columns.extend(f"{joint_name}_{suffix}" for joint_name in self.joint_order)
        return columns

    def control_callback(self, msg: JointControlData) -> None:
        if (
            len(msg.joint_torque) < self.expected_joint_count
            or len(msg.joint_position) < self.expected_joint_count
            or len(msg.joint_velocity) < self.expected_joint_count
        ):
            self.get_logger().warning("Ignoring control sample with incomplete joint arrays.")
            return

        self.latest_command = ControlSnapshot(
            joint_torque=list(msg.joint_torque[: self.expected_joint_count]),
            joint_position=list(msg.joint_position[: self.expected_joint_count]),
            joint_velocity=list(msg.joint_velocity[: self.expected_joint_count]),
        )

    def state_callback(self, msg: SimulatorStateData) -> None:
        if self.finished or self.latest_command is None:
            return

        if (
            len(msg.joint_position_values) < self.expected_joint_count
            or len(msg.joint_velocity_values) < self.expected_joint_count
        ):
            self.get_logger().warning("Ignoring state sample with incomplete joint arrays.")
            return

        simulation_time = float(msg.simulation_time)
        if self.capture_start_time is None:
            self.capture_start_time = simulation_time
            self.get_logger().info(
                f"Capture started at simulation_time={self.capture_start_time:.3f}s."
            )

        elapsed_time = simulation_time - self.capture_start_time
        if elapsed_time > self.duration:
            self.finish_capture()
            return

        torques = []
        speeds = []
        for index in range(self.expected_joint_count):
            applied_torque = self.latest_command.joint_torque[index]
            applied_torque += self.kp * (
                self.latest_command.joint_position[index] - msg.joint_position_values[index]
            )
            applied_torque += self.kd * (
                self.latest_command.joint_velocity[index] - msg.joint_velocity_values[index]
            )
            torques.append(applied_torque)
            speeds.append(msg.joint_velocity_values[index] * RPM_PER_RAD_PER_SEC)

        torque_row = [elapsed_time, simulation_time]
        torque_row.extend(torques)
        speed_row = [elapsed_time, simulation_time]
        speed_row.extend(speeds)

        self.torque_writer.writerow(torque_row)
        self.speed_writer.writerow(speed_row)
        self.torque_output_file.flush()
        self.speed_output_file.flush()
        self.samples_written += 1

        if elapsed_time >= self.duration:
            self.finish_capture()

    def destroy_node(self) -> bool:
        self.torque_output_file.close()
        self.speed_output_file.close()
        return super().destroy_node()

    def finish_capture(self) -> None:
        if self.finished:
            return

        self.finished = True
        self.get_logger().info(
            f"Capture finished. Wrote {self.samples_written} samples to "
            f"{self.torque_output_path} and {self.speed_output_path}."
        )
        rclpy.shutdown()


def main() -> int:
    args = parse_args()

    sim_config = args.sim_config
    if sim_config is None and args.robot_xml is not None:
        inferred = infer_sim_config_from_robot_xml(args.robot_xml)
        if inferred is not None:
            sim_config = inferred

    try:
        joint_order = extract_joint_order(args.robot_xml)
        kp, kd, resolved_sim_config = load_pid_gains(sim_config, args.kp, args.kd)
        torque_output_path, speed_output_path = resolve_output_paths(
            name=args.name,
            output_dir=args.output_dir,
            torque_output=args.torque_output,
            speed_output=args.speed_output,
        )
    except (FileNotFoundError, ValueError, ET.ParseError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    if resolved_sim_config is not None:
        print(f"Using simulation gains from: {resolved_sim_config}")
    elif args.kp is None or args.kd is None:
        print("Warning: no simulation config found, using zero PID gains.", file=sys.stderr)

    print(f"Logging joints in this order: {', '.join(joint_order)}")
    print(f"Capture duration: {max(args.duration, 0.001):.3f} seconds")
    print(f"Capture name: {sanitize_capture_name(args.name)}")
    print(f"Torque CSV: {torque_output_path}")
    print(f"Speed CSV: {speed_output_path}")

    rclpy.init()
    node = JointMetricCsvLogger(
        state_topic=args.state_topic,
        control_topic=args.control_topic,
        joint_order=joint_order,
        kp=kp,
        kd=kd,
        duration=args.duration,
        torque_output_path=torque_output_path,
        speed_output_path=speed_output_path,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
