#!/usr/bin/env python3

#python3 tools/combine_verify_joint_csv.py \
#  --position sim_capture/quad_mini/verify/walk_0.5_position.csv \
#  --speed sim_capture/quad_mini/verify/walk_0.5_speed.csv \
#  --output sim_capture/quad_mini/verify/walk_0.5_combined.csv

"""Combine verify position and speed CSV files into one renamed CSV."""

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Sequence


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_VERIFY_DIR = WORKSPACE_ROOT / "sim_capture/quad_mini/verify"
TIME_COLUMNS = ("elapsed_simulation_time_sec", "simulation_time_sec")
LEG_ORDER = ("FL", "FR", "HL", "HR")
JOINT_ORDER = ("hipx", "hipy", "knee")
SOURCE_LEG_BY_OUTPUT_LEG = {
    "FL": "LF",
    "FR": "RF",
    "HL": "LH",
    "HR": "RH",
}
SOURCE_JOINT_BY_OUTPUT_JOINT = {
    "hipx": "HAA",
    "hipy": "HFE",
    "knee": "KFE",
}
METRICS = (
    ("position", "position_rad"),
    ("speed", "speed_rad_per_sec"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Combine a verify position CSV and speed CSV into one output CSV with "
            "FL/FR/HL/HR and hipx/hipy/knee naming."
        )
    )
    parser.add_argument(
        "--position",
        type=Path,
        default=DEFAULT_VERIFY_DIR / "walk_0.5_position.csv",
        help="Input position CSV path.",
    )
    parser.add_argument(
        "--speed",
        type=Path,
        default=DEFAULT_VERIFY_DIR / "walk_0.5_speed.csv",
        help="Input speed CSV path.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_VERIFY_DIR / "walk_0.5_combined.csv",
        help="Output CSV path.",
    )
    return parser.parse_args()


def read_csv(path: Path) -> tuple[List[str], List[Dict[str, str]]]:
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"CSV file not found: {resolved}")

    with resolved.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV file has no header: {resolved}")
        rows = list(reader)

    if not rows:
        raise ValueError(f"CSV file is empty: {resolved}")

    return list(reader.fieldnames), rows


def validate_time_columns(position_rows: Sequence[Dict[str, str]], speed_rows: Sequence[Dict[str, str]]) -> None:
    if len(position_rows) != len(speed_rows):
        raise ValueError(
            "Position and speed CSV files do not have the same number of rows: "
            f"{len(position_rows)} != {len(speed_rows)}"
        )

    for row_index, (position_row, speed_row) in enumerate(zip(position_rows, speed_rows), start=2):
        for time_column in TIME_COLUMNS:
            if position_row.get(time_column) != speed_row.get(time_column):
                raise ValueError(
                    f"Time mismatch at CSV row {row_index} for column '{time_column}': "
                    f"{position_row.get(time_column)} != {speed_row.get(time_column)}"
                )


def validate_required_columns(position_header: Sequence[str], speed_header: Sequence[str]) -> None:
    expected_position_columns = {
        source_column_name(leg_name, joint_name, "position_rad")
        for leg_name in LEG_ORDER
        for joint_name in JOINT_ORDER
    }
    expected_speed_columns = {
        source_column_name(leg_name, joint_name, "speed_rad_per_sec")
        for leg_name in LEG_ORDER
        for joint_name in JOINT_ORDER
    }

    missing_position = sorted(expected_position_columns.difference(position_header))
    missing_speed = sorted(expected_speed_columns.difference(speed_header))

    if missing_position:
        raise ValueError(
            "Position CSV is missing expected columns: " + ", ".join(missing_position)
        )
    if missing_speed:
        raise ValueError("Speed CSV is missing expected columns: " + ", ".join(missing_speed))


def build_output_header() -> List[str]:
    header = list(TIME_COLUMNS)
    for metric_name, metric_suffix in METRICS:
        for leg_name in LEG_ORDER:
            for joint_name in JOINT_ORDER:
                header.append(f"{leg_name}_{joint_name}_{metric_suffix}")
    return header


def source_column_name(output_leg: str, output_joint: str, metric_suffix: str) -> str:
    source_leg = SOURCE_LEG_BY_OUTPUT_LEG[output_leg]
    source_joint = SOURCE_JOINT_BY_OUTPUT_JOINT[output_joint]
    return f"{source_leg}_{source_joint}_{metric_suffix}"


def build_output_rows(
    position_rows: Sequence[Dict[str, str]],
    speed_rows: Sequence[Dict[str, str]],
) -> List[List[str]]:
    output_rows: List[List[str]] = []

    for position_row, speed_row in zip(position_rows, speed_rows):
        combined_row = [position_row[time_column] for time_column in TIME_COLUMNS]

        for metric_name, metric_suffix in METRICS:
            source_row = position_row if metric_name == "position" else speed_row
            for leg_name in LEG_ORDER:
                for joint_name in JOINT_ORDER:
                    combined_row.append(source_row[source_column_name(leg_name, joint_name, metric_suffix)])

        output_rows.append(combined_row)

    return output_rows


def main() -> int:
    args = parse_args()

    position_header, position_rows = read_csv(args.position)
    speed_header, speed_rows = read_csv(args.speed)
    validate_required_columns(position_header, speed_header)
    validate_time_columns(position_rows, speed_rows)

    output_path = args.output.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(build_output_header())
        writer.writerows(build_output_rows(position_rows, speed_rows))

    print(f"Wrote combined CSV to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
