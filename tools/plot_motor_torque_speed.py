#!/usr/bin/env python3
"""Plot motor operating points from matching joint torque and speed CSV files."""

import argparse
import csv
import importlib.util
import math
import os
from pathlib import Path
import site
import sys
from typing import Dict, List, NamedTuple, Optional, Sequence, Tuple

MPL_CONFIG_DIR = Path("/tmp/quad_ocs2_matplotlib")
MPL_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CONFIG_DIR))

TIME_COLUMNS = ("elapsed_simulation_time_sec", "elapsed_time_sec", "simulation_time_sec")
TORQUE_SUFFIX = "_torque_nm"
SPEED_RAD_SUFFIX = "_speed_rad_per_sec"
SPEED_RPM_SUFFIX = "_speed_rpm"
RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * math.pi)

JOINT_NAME_MAP = {
    "HAA": "hipx",
    "HFE": "hipy",
    "KFE": "knee",
}
JOINT_DISPLAY_NAMES = {
    "hipx": "hipx",
    "hipy": "hipy",
    "knee": "knee",
}


def module_origin(module_name: str) -> Optional[Path]:
    spec = importlib.util.find_spec(module_name)
    if spec is None:
        return None
    if spec.origin is not None:
        return Path(spec.origin).resolve()
    if spec.submodule_search_locations:
        return Path(next(iter(spec.submodule_search_locations))).resolve()
    return None


def should_restart_without_user_site() -> bool:
    if sys.flags.no_user_site:
        return False

    numpy_origin = module_origin("numpy")
    matplotlib_origin = module_origin("matplotlib")
    if numpy_origin is None or matplotlib_origin is None:
        return False

    user_site = Path(site.getusersitepackages()).resolve()
    return user_site in numpy_origin.parents and user_site not in matplotlib_origin.parents


if should_restart_without_user_site():
    os.execvpe(sys.executable, [sys.executable, "-s", *sys.argv], os.environ.copy())

try:
    import matplotlib.pyplot as plt
except Exception as exc:  # pragma: no cover - environment-specific
    raise SystemExit(
        "matplotlib could not be imported. This is often caused by mixing a user-installed "
        "NumPy with the system matplotlib package.\n"
        f"Import error: {exc}\n"
        f"Try rerunning with:\n  {sys.executable} -s {' '.join(sys.argv)}"
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plot torque-vs-speed operating points for motor selection. Speed CSV values "
            "ending in _speed_rad_per_sec are converted to RPM."
        )
    )
    parser.add_argument(
        "torque_csv",
        type=Path,
        help="Torque CSV, for example walk_alt3_2.0_torque.csv.",
    )
    parser.add_argument(
        "speed_csv",
        type=Path,
        help="Speed CSV, for example walk_alt3_2.0_speed.csv.",
    )
    parser.add_argument(
        "--columns",
        nargs="+",
        default=None,
        help=(
            "Optional joints to include. Use names without suffix, such as FL_hipy, "
            "or full torque/speed column names."
        ),
    )
    parser.add_argument(
        "--signed",
        action="store_true",
        help="Plot signed RPM and signed torque instead of absolute motor demand.",
    )
    parser.add_argument(
        "--highest-torque",
        action="store_true",
        help="Plot only the joint with the highest absolute torque demand.",
    )
    parser.add_argument(
        "--max-points-per-joint",
        type=int,
        default=2500,
        help="Deterministically downsample each joint for readability; 0 keeps all points.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional image path. If omitted, the plot is shown interactively.",
    )
    parser.add_argument(
        "--title",
        default=None,
        help="Optional plot title.",
    )
    return parser.parse_args()


def load_numeric_csv(path: Path) -> Tuple[List[str], Dict[str, List[float]]]:
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"CSV file not found: {resolved}")

    with resolved.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV file has no header: {resolved}")

        columns = list(reader.fieldnames)
        values = {column: [] for column in columns}
        for row_index, row in enumerate(reader, start=2):
            try:
                for column in columns:
                    values[column].append(float(row[column]))
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Could not parse numeric value in {resolved} at CSV row {row_index}."
                ) from exc

    if not columns or not values[columns[0]]:
        raise ValueError(f"CSV file is empty: {resolved}")

    return columns, values


def strip_known_suffix(column: str) -> Optional[str]:
    for suffix in (TORQUE_SUFFIX, SPEED_RAD_SUFFIX, SPEED_RPM_SUFFIX):
        if column.endswith(suffix):
            return column[: -len(suffix)]
    return None


def speed_column_to_base_and_scale(column: str) -> Optional[Tuple[str, float]]:
    if column.endswith(SPEED_RAD_SUFFIX):
        return column[: -len(SPEED_RAD_SUFFIX)], RAD_PER_SEC_TO_RPM
    if column.endswith(SPEED_RPM_SUFFIX):
        return column[: -len(SPEED_RPM_SUFFIX)], 1.0
    return None


def normalize_requested_columns(requested_columns: Optional[Sequence[str]]) -> Optional[List[str]]:
    if requested_columns is None:
        return None

    normalized = []
    for column in requested_columns:
        base = strip_known_suffix(column)
        normalized.append(base if base is not None else column)
    return normalized


def choose_joint_bases(
    torque_columns: Sequence[str],
    speed_columns: Sequence[str],
    requested_columns: Optional[Sequence[str]],
) -> List[Tuple[str, str, str, float]]:
    torque_by_base = {
        column[: -len(TORQUE_SUFFIX)]: column
        for column in torque_columns
        if column.endswith(TORQUE_SUFFIX)
    }

    speed_by_base: Dict[str, Tuple[str, float]] = {}
    for column in speed_columns:
        parsed = speed_column_to_base_and_scale(column)
        if parsed is not None:
            base, scale = parsed
            speed_by_base[base] = (column, scale)

    common_bases = [base for base in torque_by_base if base in speed_by_base]
    if not common_bases:
        raise ValueError(
            "No matching torque/speed joint columns found. Expected names like "
            "FL_hipy_torque_nm and FL_hipy_speed_rad_per_sec."
        )

    requested_bases = normalize_requested_columns(requested_columns)
    if requested_bases is not None:
        missing = [base for base in requested_bases if base not in common_bases]
        if missing:
            available = ", ".join(common_bases)
            missing_text = ", ".join(missing)
            raise ValueError(f"Requested joints not found: {missing_text}. Available joints: {available}")
        common_bases = [base for base in common_bases if base in requested_bases]

    return [
        (base, torque_by_base[base], speed_by_base[base][0], speed_by_base[base][1])
        for base in common_bases
    ]


def format_joint_label(base: str) -> str:
    parts = base.split("_")
    if len(parts) < 2:
        return base

    leg_name = parts[0]
    joint_name = JOINT_NAME_MAP.get(parts[1], parts[1])
    joint_display_name = JOINT_DISPLAY_NAMES.get(joint_name, joint_name)
    return f"{leg_name} {joint_display_name}"


def percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return 0.0

    sorted_values = sorted(values)
    index = min(len(sorted_values) - 1, max(0, math.ceil(fraction * len(sorted_values)) - 1))
    return sorted_values[index]


def downsample_pair(
    x_values: Sequence[float],
    y_values: Sequence[float],
    max_points: int,
) -> Tuple[List[float], List[float]]:
    if max_points <= 0 or len(x_values) <= max_points:
        return list(x_values), list(y_values)

    step = math.ceil(len(x_values) / max_points)
    return list(x_values[::step]), list(y_values[::step])


class JointPlotData(NamedTuple):
    base: str
    speeds_rpm: List[float]
    torques_nm: List[float]
    abs_speeds_rpm: List[float]
    abs_torques_nm: List[float]
    summary: Tuple[str, float, float, float, float, float]


def print_summary(
    rows: Sequence[Tuple[str, float, float, float, float, float]],
) -> None:
    header = (
        "joint",
        "max_abs_rpm",
        "p95_abs_rpm",
        "max_abs_nm",
        "p95_abs_nm",
        "max_abs_power_w",
    )
    print(
        f"{header[0]:<10} {header[1]:>12} {header[2]:>12} "
        f"{header[3]:>11} {header[4]:>11} {header[5]:>15}"
    )
    print("-" * 78)
    for joint, max_rpm, p95_rpm, max_torque, p95_torque, max_power in rows:
        print(
            f"{joint:<10} {max_rpm:12.1f} {p95_rpm:12.1f} "
            f"{max_torque:11.2f} {p95_torque:11.2f} {max_power:15.1f}"
        )


def plot_motor_points(
    torque_csv: Path,
    speed_csv: Path,
    requested_columns: Optional[Sequence[str]],
    signed: bool,
    highest_torque: bool,
    max_points_per_joint: int,
    output: Optional[Path],
    title: Optional[str],
) -> Optional[Path]:
    torque_columns, torque_values = load_numeric_csv(torque_csv)
    speed_columns, speed_values = load_numeric_csv(speed_csv)
    joint_columns = choose_joint_bases(torque_columns, speed_columns, requested_columns)

    plot_data: List[JointPlotData] = []

    for base, torque_column, speed_column, speed_scale in joint_columns:
        torques_nm = torque_values[torque_column]
        speeds_rpm = [speed * speed_scale for speed in speed_values[speed_column]]

        if len(torques_nm) != len(speeds_rpm):
            raise ValueError(
                f"Column length mismatch for {base}: "
                f"{len(torques_nm)} torque samples vs {len(speeds_rpm)} speed samples."
            )

        abs_speeds_rpm = [abs(speed) for speed in speeds_rpm]
        abs_torques_nm = [abs(torque) for torque in torques_nm]
        powers_w = [
            abs(torque * (speed_rpm / RAD_PER_SEC_TO_RPM))
            for torque, speed_rpm in zip(torques_nm, speeds_rpm)
        ]

        summary = (
            base,
            max(abs_speeds_rpm),
            percentile(abs_speeds_rpm, 0.95),
            max(abs_torques_nm),
            percentile(abs_torques_nm, 0.95),
            max(powers_w),
        )
        plot_data.append(
            JointPlotData(
                base=base,
                speeds_rpm=speeds_rpm,
                torques_nm=torques_nm,
                abs_speeds_rpm=abs_speeds_rpm,
                abs_torques_nm=abs_torques_nm,
                summary=summary,
            )
        )

    plot_data.sort(key=lambda data: (data.summary[3], data.summary[1]), reverse=True)
    summary_rows = [data.summary for data in plot_data]
    print_summary(summary_rows)

    if highest_torque:
        plot_data = plot_data[:1]
        title = title or f"Highest Torque Joint: {format_joint_label(plot_data[0].base)}"

    figure, axis = plt.subplots(figsize=(11.5, 7.0))

    for data in plot_data:
        x_values = data.speeds_rpm if signed else data.abs_speeds_rpm
        y_values = data.torques_nm if signed else data.abs_torques_nm
        plot_x, plot_y = downsample_pair(x_values, y_values, max_points_per_joint)
        axis.scatter(
            plot_x,
            plot_y,
            s=10 if highest_torque else 7,
            alpha=0.34 if highest_torque else 0.28,
            linewidths=0,
            label=format_joint_label(data.base),
        )

        if highest_torque:
            axis.annotate(
                f"max {data.summary[3]:.2f} Nm\n{data.summary[1]:.1f} rpm",
                xy=(data.summary[1], data.summary[3]),
                xytext=(8, 8),
                textcoords="offset points",
            )

    if highest_torque:
        print(f"\nPlotting highest-torque joint only: {plot_data[0].base}")
    else:
        print("\nPlotting all matched joints.")

    axis.grid(True, alpha=0.3)
    axis.set_xlabel("motor speed [rpm]")
    axis.set_ylabel("motor torque [Nm]")
    if signed:
        axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.45)
        axis.axvline(0.0, color="black", linewidth=0.8, alpha=0.45)
    else:
        axis.set_xlim(left=0.0)
        axis.set_ylim(bottom=0.0)

    mode_text = "signed operating points" if signed else "absolute motor demand"
    axis.set_title(title or f"Motor Torque-Speed Plot ({mode_text})")
    axis.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False)
    figure.tight_layout()

    if output is None:
        plt.show()
        return None

    resolved_output = output.expanduser().resolve()
    resolved_output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(resolved_output, dpi=170, bbox_inches="tight")
    print(f"Saved plot to {resolved_output}")
    return resolved_output


def main() -> int:
    args = parse_args()
    plot_motor_points(
        torque_csv=args.torque_csv,
        speed_csv=args.speed_csv,
        requested_columns=args.columns,
        signed=args.signed,
        highest_torque=args.highest_torque,
        max_points_per_joint=args.max_points_per_joint,
        output=args.output,
        title=args.title,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
