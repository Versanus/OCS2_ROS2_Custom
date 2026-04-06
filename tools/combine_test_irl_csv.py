#!/usr/bin/env python3

"""Combine quad_mini test_irl position, speed, and torque CSVs into *_comb.csv files."""
#python3 tools/combine_test_irl_csv.py --input-dir sim_capture/quad_mini/test_irl

import argparse
import csv
from pathlib import Path
from typing import Dict, Iterable, List, Sequence


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT_DIR = WORKSPACE_ROOT / "sim_capture/quad_mini/test_irl"
TIME_COLUMN = "simulation_time_sec"
TIME_COLUMNS_TO_VALIDATE = ("elapsed_simulation_time_sec", "simulation_time_sec")
JOINT_NAMES = (
    "FL_HipX",
    "FL_HipY",
    "FL_Knee",
    "FR_HipX",
    "FR_HipY",
    "FR_Knee",
    "HL_HipX",
    "HL_HipY",
    "HL_Knee",
    "HR_HipX",
    "HR_HipY",
    "HR_Knee",
)
POSITION_SOURCE_COLUMNS = (
    "FL_hipx_position_rad",
    "FL_hipy_position_rad",
    "FL_knee_position_rad",
    "FR_hipx_position_rad",
    "FR_hipy_position_rad",
    "FR_knee_position_rad",
    "HL_hipx_position_rad",
    "HL_hipy_position_rad",
    "HL_knee_position_rad",
    "HR_hipx_position_rad",
    "HR_hipy_position_rad",
    "HR_knee_position_rad",
)
SPEED_SOURCE_COLUMNS = (
    "FL_hipx_speed_rad_per_sec",
    "FL_hipy_speed_rad_per_sec",
    "FL_knee_speed_rad_per_sec",
    "FR_hipx_speed_rad_per_sec",
    "FR_hipy_speed_rad_per_sec",
    "FR_knee_speed_rad_per_sec",
    "HL_hipx_speed_rad_per_sec",
    "HL_hipy_speed_rad_per_sec",
    "HL_knee_speed_rad_per_sec",
    "HR_hipx_speed_rad_per_sec",
    "HR_hipy_speed_rad_per_sec",
    "HR_knee_speed_rad_per_sec",
)
TORQUE_SOURCE_COLUMNS = (
    "FL_hipx_torque_nm",
    "FL_hipy_torque_nm",
    "FL_knee_torque_nm",
    "FR_hipx_torque_nm",
    "FR_hipy_torque_nm",
    "FR_knee_torque_nm",
    "HL_hipx_torque_nm",
    "HL_hipy_torque_nm",
    "HL_knee_torque_nm",
    "HR_hipx_torque_nm",
    "HR_hipy_torque_nm",
    "HR_knee_torque_nm",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Combine all *_position.csv, *_speed.csv, and *_torque.csv triples in "
            "quad_mini/test_irl into *_comb.csv outputs with sim_time, *_pos, "
            "*_vel, and *_torque columns."
        )
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=DEFAULT_INPUT_DIR,
        help="Directory containing *_position.csv and *_speed.csv files.",
    )
    return parser.parse_args()


def read_csv(path: Path) -> tuple[List[str], List[Dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV file has no header: {path}")
        rows = list(reader)

    if not rows:
        raise ValueError(f"CSV file is empty: {path}")

    return list(reader.fieldnames), rows


def validate_required_columns(header: Sequence[str], required_columns: Iterable[str], path: Path) -> None:
    missing_columns = [column for column in required_columns if column not in header]
    if missing_columns:
        missing_text = ", ".join(missing_columns)
        raise ValueError(f"CSV file is missing required columns in {path}: {missing_text}")


def validate_matching_rows(
    position_rows: Sequence[Dict[str, str]],
    speed_rows: Sequence[Dict[str, str]],
    torque_rows: Sequence[Dict[str, str]],
    position_path: Path,
    speed_path: Path,
    torque_path: Path,
) -> None:
    if len(position_rows) != len(speed_rows) or len(position_rows) != len(torque_rows):
        raise ValueError(
            "Position, speed, and torque CSV files do not have the same number of rows: "
            f"{position_path} ({len(position_rows)}), "
            f"{speed_path} ({len(speed_rows)}), "
            f"{torque_path} ({len(torque_rows)})"
        )

    for row_index, (position_row, speed_row, torque_row) in enumerate(
        zip(position_rows, speed_rows, torque_rows),
        start=2,
    ):
        for column_name in TIME_COLUMNS_TO_VALIDATE:
            if position_row[column_name] != speed_row[column_name] or position_row[column_name] != torque_row[column_name]:
                raise ValueError(
                    f"Time mismatch at CSV row {row_index} for '{column_name}': "
                    f"{position_path} has {position_row[column_name]!r} and "
                    f"{speed_path} has {speed_row[column_name]!r} and "
                    f"{torque_path} has {torque_row[column_name]!r}"
                )


def build_output_header() -> List[str]:
    header = ["sim_time"]
    header.extend(f"{joint_name}_pos" for joint_name in JOINT_NAMES)
    header.extend(f"{joint_name}_vel" for joint_name in JOINT_NAMES)
    header.extend(f"{joint_name}_torque" for joint_name in JOINT_NAMES)
    return header


def build_output_rows(
    position_rows: Sequence[Dict[str, str]],
    speed_rows: Sequence[Dict[str, str]],
    torque_rows: Sequence[Dict[str, str]],
) -> List[List[str]]:
    output_rows: List[List[str]] = []

    for position_row, speed_row, torque_row in zip(position_rows, speed_rows, torque_rows):
        row = [position_row[TIME_COLUMN]]
        row.extend(position_row[column_name] for column_name in POSITION_SOURCE_COLUMNS)
        row.extend(speed_row[column_name] for column_name in SPEED_SOURCE_COLUMNS)
        row.extend(torque_row[column_name] for column_name in TORQUE_SOURCE_COLUMNS)
        output_rows.append(row)

    return output_rows


def combine_pair(position_path: Path, speed_path: Path, torque_path: Path, output_path: Path) -> None:
    position_header, position_rows = read_csv(position_path)
    speed_header, speed_rows = read_csv(speed_path)
    torque_header, torque_rows = read_csv(torque_path)

    validate_required_columns(position_header, (TIME_COLUMN, *TIME_COLUMNS_TO_VALIDATE, *POSITION_SOURCE_COLUMNS), position_path)
    validate_required_columns(speed_header, (TIME_COLUMN, *TIME_COLUMNS_TO_VALIDATE, *SPEED_SOURCE_COLUMNS), speed_path)
    validate_required_columns(torque_header, (TIME_COLUMN, *TIME_COLUMNS_TO_VALIDATE, *TORQUE_SOURCE_COLUMNS), torque_path)
    validate_matching_rows(position_rows, speed_rows, torque_rows, position_path, speed_path, torque_path)

    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(build_output_header())
        writer.writerows(build_output_rows(position_rows, speed_rows, torque_rows))


def main() -> int:
    args = parse_args()
    input_dir = args.input_dir.expanduser().resolve()

    if not input_dir.is_dir():
        raise FileNotFoundError(f"Input directory not found: {input_dir}")

    position_files = sorted(input_dir.glob("*_position.csv"))
    if not position_files:
        raise FileNotFoundError(f"No *_position.csv files found in {input_dir}")

    combined_count = 0
    for position_path in position_files:
        base_name = position_path.name[: -len("_position.csv")]
        speed_path = input_dir / f"{base_name}_speed.csv"
        torque_path = input_dir / f"{base_name}_torque.csv"
        if not speed_path.exists():
            raise FileNotFoundError(f"Missing matching speed CSV for {position_path.name}: {speed_path.name}")
        if not torque_path.exists():
            raise FileNotFoundError(f"Missing matching torque CSV for {position_path.name}: {torque_path.name}")

        output_path = input_dir / f"{base_name}_comb.csv"
        combine_pair(position_path, speed_path, torque_path, output_path)
        combined_count += 1
        print(f"Wrote {output_path}")

    print(f"Combined {combined_count} CSV pair(s) in {input_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
