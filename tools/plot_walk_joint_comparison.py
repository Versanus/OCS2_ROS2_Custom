#!/usr/bin/env python3

"""Compare one joint across walk captures at multiple commanded speeds."""
#python3 tools/plot_walk_joint_comparison.py \
#  --input-dir sim_capture/quad_mini/new \
#  --leg FL \
#  --joint hipx

import argparse
import csv
import importlib.util
import os
from pathlib import Path
import re
import site
import sys
from typing import Dict, List, Optional, Tuple

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_DIR = WORKSPACE_ROOT / "sim_capture/quad_mini/new"
MPL_CONFIG_DIR = Path("/tmp/quad_ocs2_matplotlib")
MPL_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CONFIG_DIR))

TIME_COLUMNS = ("elapsed_simulation_time_sec", "elapsed_time_sec", "simulation_time_sec")
LEGS = ("FL", "HL", "FR", "HR")
JOINT_NAMES = ("HAA", "HFE", "KFE")
JOINT_LABELS = {
    "HAA": "hipx",
    "HFE": "hipy",
    "KFE": "knee",
}
METRICS = {
    "torque": {
        "suffix": "torque_nm",
        "ylabel": "torque [Nm]",
        "default_output_name": "walk_joint_torque_comparison.png",
    },
    "speed": {
        "suffix": "speed_rad_per_sec",
        "ylabel": "speed [rad/s]",
        "default_output_name": "walk_joint_speed_comparison.png",
    },
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
            "Plot one joint from walk_0.5, walk_1.0, walk_1.5, and walk_2.0 captures. "
            "Creates separate torque and speed figures."
        )
    )
    parser.add_argument(
        "--leg",
        choices=LEGS,
        default="LF",
        help="Leg containing the joint to compare.",
    )
    parser.add_argument(
        "--joint",
        default="HAA",
        help="Joint name to compare. Accepts HAA/HFE/KFE or hipx/hipy/knee.",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help="Directory containing walk_<speed>_torque.csv and walk_<speed>_speed.csv files.",
    )
    parser.add_argument(
        "--time-column",
        choices=TIME_COLUMNS,
        default="elapsed_simulation_time_sec",
        help="Time axis to use.",
    )
    parser.add_argument(
        "--torque-output",
        type=Path,
        default=None,
        help="Optional output image path for the torque figure.",
    )
    parser.add_argument(
        "--speed-output",
        type=Path,
        default=None,
        help="Optional output image path for the speed figure.",
    )
    parser.add_argument(
        "--title-prefix",
        default="Walk Comparison",
        help="Optional prefix used in figure titles.",
    )
    return parser.parse_args()


def normalize_joint_name(joint_name: str) -> str:
    normalized = joint_name.strip().upper()
    aliases = {
        "HIPX": "HAA",
        "HIPY": "HFE",
        "KNEE": "KFE",
    }
    normalized = aliases.get(normalized, normalized)
    if normalized not in JOINT_NAMES:
        choices = ", ".join([*JOINT_NAMES, "hipx", "hipy", "knee"])
        raise ValueError(f"Unsupported joint '{joint_name}'. Use one of: {choices}.")
    return normalized


def load_csv(path: Path, time_column: str) -> Tuple[List[float], Dict[str, List[float]]]:
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"CSV file not found: {resolved}")

    with resolved.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV file has no header: {resolved}")
        if time_column not in reader.fieldnames:
            raise ValueError(f"{resolved} is missing time column '{time_column}'.")

        data_columns = [name for name in reader.fieldnames if name not in TIME_COLUMNS]
        time_values: List[float] = []
        values = {column: [] for column in data_columns}

        for row_index, row in enumerate(reader, start=2):
            try:
                time_values.append(float(row[time_column]))
                for column in data_columns:
                    values[column].append(float(row[column]))
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Could not parse numeric value in {resolved} at CSV row {row_index}."
                ) from exc

    if not time_values:
        raise ValueError(f"CSV file is empty: {resolved}")

    return time_values, values


def metric_file(input_dir: Path, walk_speed: str, metric_name: str) -> Path:
    return input_dir.expanduser().resolve() / f"walk_{walk_speed}_{metric_name}.csv"


def discover_walk_speeds(input_dir: Path, metric_name: str) -> List[str]:
    pattern = re.compile(rf"walk_(.+)_{re.escape(metric_name)}\.csv$")
    discovered = []
    for path in sorted(input_dir.expanduser().resolve().glob(f"walk_*_{metric_name}.csv")):
        match = pattern.match(path.name)
        if match:
            discovered.append(match.group(1))
    if not discovered:
        raise FileNotFoundError(
            f"No walk capture files matching 'walk_*_{metric_name}.csv' were found in {input_dir.expanduser().resolve()}."
        )
    return discovered


def metric_column(leg_name: str, joint_name: str, metric_name: str) -> str:
    normalized_joint_name = JOINT_LABELS.get(joint_name, joint_name.lower())
    return f"{leg_name}_{normalized_joint_name}_{METRICS[metric_name]['suffix']}"


def plot_metric(
    leg_name: str,
    joint_name: str,
    input_dir: Path,
    time_column: str,
    metric_name: str,
    output_path: Optional[Path],
    title_prefix: str,
) -> Optional[Path]:
    metric_config = METRICS[metric_name]
    column_name = metric_column(leg_name, joint_name, metric_name)
    walk_speeds = discover_walk_speeds(input_dir, metric_name)

    figure, axis = plt.subplots(1, 1, figsize=(10.0, 4.8))

    for walk_speed in walk_speeds:
        csv_path = metric_file(input_dir, walk_speed, metric_name)
        time_values, values = load_csv(csv_path, time_column)
        if column_name not in values:
            raise ValueError(f"{csv_path} is missing expected column '{column_name}'.")
        axis.plot(time_values, values[column_name], linewidth=1.5, label=f"walk {walk_speed}")

    joint_label = JOINT_LABELS[joint_name]
    axis.set_title(f"{leg_name} {joint_label}")
    axis.set_ylabel(metric_config["ylabel"])
    axis.set_xlabel(time_column)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper right")
    figure.suptitle(f"{title_prefix} - {leg_name} {joint_label} {metric_name.capitalize()}")
    figure.tight_layout()

    if output_path is None:
        return None

    resolved_output = output_path.expanduser().resolve()
    resolved_output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(resolved_output, dpi=160, bbox_inches="tight")
    print(f"Saved {metric_name} plot to {resolved_output}")
    return resolved_output


def main() -> int:
    args = parse_args()

    try:
        joint_name = normalize_joint_name(args.joint)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    torque_path = plot_metric(
        leg_name=args.leg,
        joint_name=joint_name,
        input_dir=args.input_dir,
        time_column=args.time_column,
        metric_name="torque",
        output_path=args.torque_output,
        title_prefix=args.title_prefix,
    )
    speed_path = plot_metric(
        leg_name=args.leg,
        joint_name=joint_name,
        input_dir=args.input_dir,
        time_column=args.time_column,
        metric_name="speed",
        output_path=args.speed_output,
        title_prefix=args.title_prefix,
    )

    if torque_path is None or speed_path is None:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
