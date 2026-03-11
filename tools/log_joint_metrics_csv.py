#!/usr/bin/env python3

"""Write published joint torques and velocities from ROS directly to CSV files."""

import argparse
import csv
import re
import sys
from pathlib import Path
from typing import List, Optional, Sequence

try:
    import rclpy
    from rclpy.node import Node

    from legged_msgs.msg import JointControlData, SimulatorStateData
except ImportError as exc:  # pragma: no cover - environment-specific
    raise SystemExit(
        "ROS 2 Python packages were not found. Source your workspace first:\n"
        "  source /home/kaan/quad_ocs2_ws/install/setup.bash"
    ) from exc


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_JOINT_COUNT = 12
DEFAULT_JOINT_NAMES = [
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture the published joint_torque and joint_velocity arrays from a "
            "ROS topic and save them directly to separate CSV files."
        )
    )
    parser.add_argument(
        "--state-topic",
        default="simulator_state_data",
        help="ROS topic carrying SimulatorStateData messages.",
    )
    parser.add_argument(
        "--control-topic",
        default="joint_control_data",
        help="ROS topic carrying JointControlData messages.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Capture duration in seconds, starting from the first valid message.",
    )
    parser.add_argument(
        "--name",
        default="joint_command",
        help="Capture name used in the default CSV filename.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=WORKSPACE_ROOT,
        help="Directory for the default CSV files.",
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
    parser.add_argument(
        "--joint-count",
        type=int,
        default=DEFAULT_JOINT_COUNT,
        help="Expected number of joint torques to log from each message.",
    )
    parser.add_argument(
        "--joint-names",
        default=",".join(DEFAULT_JOINT_NAMES),
        help=(
            "Comma-separated joint names used for CSV headers. "
            "Defaults to the B2 MuJoCo order."
        ),
    )
    return parser.parse_args()


def sanitize_capture_name(name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("._-")
    return sanitized or "joint_torque"


def resolve_output_paths(
    name: str,
    output_dir: Path,
    torque_output: Optional[Path],
    speed_output: Optional[Path],
) -> tuple[Path, Path]:
    if torque_output is not None and speed_output is not None:
        return torque_output.expanduser().resolve(), speed_output.expanduser().resolve()

    sanitized_name = sanitize_capture_name(name)
    resolved_dir = output_dir.expanduser().resolve()
    default_torque = resolved_dir / f"{sanitized_name}_torque.csv"
    default_speed = resolved_dir / f"{sanitized_name}_speed.csv"
    resolved_torque = (
        torque_output.expanduser().resolve() if torque_output is not None else default_torque
    )
    resolved_speed = (
        speed_output.expanduser().resolve() if speed_output is not None else default_speed
    )
    return resolved_torque, resolved_speed


def parse_joint_names(joint_names_text: str, joint_count: int) -> List[str]:
    joint_names = [name.strip() for name in joint_names_text.split(",") if name.strip()]
    if len(joint_names) != joint_count:
        raise ValueError(
            f"Expected {joint_count} joint names, but got {len(joint_names)}."
        )
    return joint_names


class JointTorqueCsvLogger(Node):
    def __init__(
        self,
        state_topic: str,
        control_topic: str,
        joint_names: Sequence[str],
        duration: float,
        torque_output_path: Path,
        speed_output_path: Path,
    ) -> None:
        super().__init__("joint_torque_csv_logger")
        self.joint_names = list(joint_names)
        self.joint_count = len(self.joint_names)
        self.duration = max(duration, 0.001)
        self.capture_start_time: Optional[float] = None
        self.latest_control: Optional[JointControlData] = None
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
        self.speed_writer.writerow(self._header("speed_rad_per_sec"))
        self.torque_output_file.flush()
        self.speed_output_file.flush()

        self.create_subscription(JointControlData, control_topic, self.control_callback, 10)
        self.create_subscription(SimulatorStateData, state_topic, self.state_callback, 10)
        self.get_logger().info(
            "Ready. Waiting for a valid control/state pair to start a "
            f"{self.duration:.3f}s capture."
        )

    def _header(self, suffix: str) -> List[str]:
        columns = ["elapsed_simulation_time_sec", "simulation_time_sec"]
        columns.extend(f"{joint_name}_{suffix}" for joint_name in self.joint_names)
        return columns

    def control_callback(self, msg: JointControlData) -> None:
        if self.finished:
            return

        if len(msg.joint_torque) < self.joint_count or len(msg.joint_velocity) < self.joint_count:
            self.get_logger().warning(
                "Ignoring JointControlData message with incomplete torque/velocity arrays."
            )
            return

        self.latest_control = msg

    def state_callback(self, msg: SimulatorStateData) -> None:
        if self.finished or self.latest_control is None:
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

        torque_row = [elapsed_time, simulation_time]
        torque_row.extend(self.latest_control.joint_torque[: self.joint_count])
        speed_row = [elapsed_time, simulation_time]
        speed_row.extend(self.latest_control.joint_velocity[: self.joint_count])

        self.torque_writer.writerow(torque_row)
        self.speed_writer.writerow(speed_row)
        self.torque_output_file.flush()
        self.speed_output_file.flush()
        self.latest_control = None
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
    joint_count = max(args.joint_count, 1)

    try:
        joint_names = parse_joint_names(args.joint_names, joint_count)
        torque_output_path, speed_output_path = resolve_output_paths(
            args.name,
            args.output_dir,
            args.torque_output,
            args.speed_output,
        )
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Control topic: {args.control_topic}")
    print(f"State topic: {args.state_topic}")
    print(f"Joint count: {joint_count}")
    print(f"Joint names: {', '.join(joint_names)}")
    print(f"Capture duration: {max(args.duration, 0.001):.3f} seconds")
    print(f"Torque CSV: {torque_output_path}")
    print(f"Speed CSV: {speed_output_path}")

    rclpy.init()
    node = JointTorqueCsvLogger(
        state_topic=args.state_topic,
        control_topic=args.control_topic,
        joint_names=joint_names,
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
