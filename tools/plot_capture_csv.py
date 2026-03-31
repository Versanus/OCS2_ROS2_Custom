#!/usr/bin/env python3
#python3 tools/plot_capture_csv.py sim_capture/quad_mini/new/stand_position.csv
"""Plot recorded joint capture CSV files."""

import argparse
import csv
import importlib.util
import math
import os
from pathlib import Path
import site
import sys
from typing import Dict, List, Optional, Sequence, Tuple

WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
MPL_CONFIG_DIR = Path("/tmp/quad_ocs2_matplotlib")
MPL_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CONFIG_DIR))

TIME_COLUMNS = ("elapsed_simulation_time_sec", "elapsed_time_sec", "simulation_time_sec")
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
            "Plot joint capture CSV files such as sim_capture/b2/trot_speed.csv "
            "or sim_capture/b2/trot_torque.csv."
        )
    )
    parser.add_argument(
        "csv_files",
        nargs="+",
        type=Path,
        help="One or more CSV files with the same data columns.",
    )
    parser.add_argument(
        "--time-column",
        choices=TIME_COLUMNS,
        default="elapsed_simulation_time_sec",
        help="Time axis to use.",
    )
    parser.add_argument(
        "--columns",
        nargs="+",
        default=None,
        help="Optional subset of data columns to plot.",
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


def load_csv(path: Path, time_column: str) -> Tuple[List[str], List[float], Dict[str, List[float]]]:
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

    return data_columns, time_values, values


def choose_columns(
    requested_columns: Optional[Sequence[str]],
    common_columns: Sequence[str],
) -> List[str]:
    if requested_columns is None:
        return list(common_columns)

    missing = [column for column in requested_columns if column not in common_columns]
    if missing:
        available = ", ".join(common_columns)
        missing_text = ", ".join(missing)
        raise ValueError(f"Requested columns not found: {missing_text}. Available columns: {available}")

    return list(requested_columns)


def build_grid(column_count: int) -> Tuple[int, int]:
    cols = min(3, max(1, math.ceil(math.sqrt(column_count))))
    rows = math.ceil(column_count / cols)
    return rows, cols


def infer_ylabel(columns: Sequence[str]) -> str:
    if not columns:
        return "value"

    if all(column.endswith("_speed_rpm") for column in columns):
        return "speed [rpm]"
    if all(column.endswith("_speed_rad_per_sec") for column in columns):
        return "speed [rad/s]"
    if all(column.endswith("_torque_nm") for column in columns):
        return "torque [Nm]"
    if all(column.endswith("_position_rad") for column in columns):
        return "position [rad]"
    return "value"


def format_column_label(column: str) -> str:
    parts = column.split("_")
    if len(parts) < 3:
        return column

    leg_name = parts[0]
    joint_name = parts[1]
    metric_suffix = parts[-1]

    # Support both old names like FL_HAA_torque_nm and new names like FL_hipx_torque_nm.
    joint_name = JOINT_NAME_MAP.get(joint_name, joint_name)
    joint_display_name = JOINT_DISPLAY_NAMES.get(joint_name)
    if joint_display_name is None:
        return column

    if metric_suffix == "nm":
        metric_name = "torque"
    elif metric_suffix == "sec":
        metric_name = "speed"
    elif metric_suffix == "rad":
        metric_name = "position"
    else:
        metric_name = None

    if metric_name is None:
        return f"{leg_name} {joint_display_name}"
    return f"{leg_name} {joint_display_name} {metric_name}"


def plot_files(
    csv_files: Sequence[Path],
    time_column: str,
    requested_columns: Optional[Sequence[str]],
    output: Optional[Path],
    title: Optional[str],
) -> Optional[Path]:
    loaded = []
    common_columns = None

    for csv_path in csv_files:
        columns, time_values, values = load_csv(csv_path, time_column)
        loaded.append((csv_path.expanduser().resolve(), time_values, values, columns))
        common_columns = set(columns) if common_columns is None else common_columns.intersection(columns)

    assert common_columns is not None
    if not common_columns:
        raise ValueError(
            "The selected CSV files do not share any plottable columns. "
            "Use files with the same metric type, for example only speed CSVs or only torque CSVs."
        )

    ordered_common_columns = [column for column in loaded[0][3] if column in common_columns]
    plot_columns = choose_columns(requested_columns, ordered_common_columns)

    rows, cols = build_grid(len(plot_columns))
    figure, axes = plt.subplots(rows, cols, figsize=(5.0 * cols, 2.8 * rows), sharex=True)
    axis_list = list(axes.flat) if hasattr(axes, "flat") else [axes]

    for axis, column in zip(axis_list, plot_columns):
        for csv_path, time_values, values, _ in loaded:
            axis.plot(time_values, values[column], linewidth=1.2, label=csv_path.stem)
        axis.set_title(format_column_label(column))
        axis.grid(True, alpha=0.3)

    for axis in axis_list[len(plot_columns) :]:
        axis.remove()

    ylabel = infer_ylabel(plot_columns)
    for index, axis in enumerate(axis_list[: len(plot_columns)]):
        if index % cols == 0:
            axis.set_ylabel(ylabel)
        if index >= len(plot_columns) - cols:
            axis.set_xlabel(time_column)

    if len(loaded) > 1:
        axis_list[0].legend(loc="upper right")

    figure.suptitle(title or "Joint Capture CSV Plot")
    figure.tight_layout()

    if output is None:
        plt.show()
        return None

    resolved_output = output.expanduser().resolve()
    resolved_output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(resolved_output, dpi=160, bbox_inches="tight")
    print(f"Saved plot to {resolved_output}")
    return resolved_output


def main() -> int:
    args = parse_args()
    plot_files(
        csv_files=args.csv_files,
        time_column=args.time_column,
        requested_columns=args.columns,
        output=args.output,
        title=args.title,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
