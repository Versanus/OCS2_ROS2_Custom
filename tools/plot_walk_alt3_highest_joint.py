#!/usr/bin/env python3
"""Plot torque-vs-speed demand for the highest-torque joint in walk_alt3_2.0."""

import csv
import importlib.util
import math
import os
from pathlib import Path
import site
import sys

MPL_CONFIG_DIR = Path("/tmp/quad_ocs2_matplotlib")
MPL_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(MPL_CONFIG_DIR))


def module_origin(module_name):
    spec = importlib.util.find_spec(module_name)
    if spec is None:
        return None
    if spec.origin is not None:
        return Path(spec.origin).resolve()
    if spec.submodule_search_locations:
        return Path(next(iter(spec.submodule_search_locations))).resolve()
    return None


def should_restart_without_user_site():
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
except Exception as exc:
    raise SystemExit(
        "matplotlib could not be imported. Try rerunning with:\n"
        f"  {sys.executable} -s {' '.join(sys.argv)}\n"
        f"Import error: {exc}"
    ) from exc

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "sim_capture" / "a1_custom" / "limitless"
TORQUE_CSV = DATA_DIR / "walk_alt3_2.0_torque.csv"
SPEED_CSV = DATA_DIR / "walk_alt3_2.0_speed.csv"
OUTPUT = DATA_DIR / "walk_alt3_2.0_highest_joint_torque_x_speed_y.png"
MOTOR_STYLE_OUTPUT = DATA_DIR / "walk_alt3_2.0_highest_joint_motor_style.png"
TURKISH_OUTPUT = DATA_DIR / "walk_alt3_2.0_motor_secimi_1_1_tr.png"
TURKISH_KNEE_OUTPUT = DATA_DIR / "walk_alt3_2.0_knee_motor_secimi_1_1_tr.png"

TIME_COLUMN = "elapsed_simulation_time_sec"
TORQUE_SUFFIX = "_torque_nm"
SPEED_SUFFIX = "_speed_rad_per_sec"
RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * math.pi)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")

        data = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name in reader.fieldnames:
                data[name].append(float(row[name]))

    return data


def percentile(values, fraction):
    sorted_values = sorted(values)
    index = min(len(sorted_values) - 1, max(0, math.ceil(fraction * len(sorted_values)) - 1))
    return sorted_values[index]


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def recommended_no_load_speed(abs_torque_nm, abs_speed_rpm, stall_torque):
    required = []
    for torque, speed in zip(abs_torque_nm, abs_speed_rpm):
        if torque >= stall_torque:
            continue
        required.append(speed / (1.0 - torque / stall_torque))
    return max(required)


def main():
    torque_data = read_csv(TORQUE_CSV)
    speed_data = read_csv(SPEED_CSV)

    torque_columns = [name for name in torque_data if name.endswith(TORQUE_SUFFIX)]
    highest_column = max(
        torque_columns,
        key=lambda name: max(abs(value) for value in torque_data[name]),
    )
    joint = highest_column[: -len(TORQUE_SUFFIX)]
    speed_column = f"{joint}{SPEED_SUFFIX}"

    if speed_column not in speed_data:
        raise ValueError(f"Missing matching speed column: {speed_column}")

    speed_rpm = [value * RAD_PER_SEC_TO_RPM for value in speed_data[speed_column]]
    torque_nm = torque_data[highest_column]
    abs_speed_rpm = [abs(value) for value in speed_rpm]
    abs_torque_nm = [abs(value) for value in torque_nm]
    power_w = [
        abs(torque * (speed / RAD_PER_SEC_TO_RPM))
        for torque, speed in zip(torque_nm, speed_rpm)
    ]

    max_abs_speed = max(abs_speed_rpm)
    max_abs_torque = max(abs_torque_nm)
    max_power = max(power_w)
    p95_speed = percentile(abs_speed_rpm, 0.95)
    p95_torque = percentile(abs_torque_nm, 0.95)
    rms_speed = rms(speed_rpm)
    rms_torque = rms(torque_nm)
    max_torque_index = max(range(len(abs_torque_nm)), key=lambda index: abs_torque_nm[index])
    max_speed_index = max(range(len(abs_speed_rpm)), key=lambda index: abs_speed_rpm[index])
    max_power_index = max(range(len(power_w)), key=lambda index: power_w[index])

    recommended_stall_torque = max_abs_torque * 1.40
    recommended_free_speed = recommended_no_load_speed(
        abs_torque_nm,
        abs_speed_rpm,
        recommended_stall_torque,
    ) * 1.10
    curve_torque = [
        recommended_stall_torque * index / 100.0
        for index in range(101)
    ]
    curve_speed = [
        recommended_free_speed * (1.0 - torque / recommended_stall_torque)
        for torque in curve_torque
    ]

    figure, axis = plt.subplots(figsize=(9.0, 7.0))
    axis.scatter(
        abs_torque_nm,
        abs_speed_rpm,
        s=7,
        alpha=0.18,
        linewidths=0,
        color="tab:blue",
        label="simülasyon noktaları",
    )
    axis.plot(
        curve_torque,
        curve_speed,
        color="tab:red",
        linewidth=2.3,
        label="önerilen 1:1 motor eğrisi",
    )
    axis.fill_between(curve_torque, curve_speed, 0.0, color="tab:red", alpha=0.08)
    axis.scatter([max_abs_torque], [max_abs_speed], s=55, color="tab:red", label="gerekli sınır")
    axis.scatter(
        [abs_torque_nm[max_power_index]],
        [abs_speed_rpm[max_power_index]],
        s=45,
        color="black",
        label="maks. güç noktası",
    )
    axis.axvline(max_abs_torque, color="tab:red", linewidth=1.0, alpha=0.65)
    axis.axhline(max_abs_speed, color="tab:red", linewidth=1.0, alpha=0.65)
    axis.axvline(p95_torque, color="tab:orange", linewidth=1.0, alpha=0.8, linestyle="--")
    axis.axhline(p95_speed, color="tab:orange", linewidth=1.0, alpha=0.8, linestyle="--", label="%95 talep")
    axis.grid(True, alpha=0.3)
    axis.set_xlim(left=0.0)
    axis.set_ylim(bottom=0.0)
    axis.set_xlabel("Tork [Nm]")
    axis.set_ylabel("Hız [rpm]")
    axis.set_title(
        f"{joint} motor seçimi, 1:1 direkt sürüş "
        f"({max_abs_torque:.2f} Nm, {max_abs_speed:.1f} rpm)"
    )
    axis.text(
        0.04,
        0.96,
        "Öneri:\n"
        f"tepe tork >= {recommended_stall_torque:.1f} Nm\n"
        f"boşta hız >= {recommended_free_speed:.0f} rpm\n"
        "sürekli tork >= 9-10 Nm\n"
        "tepe güç >= 180-220 W",
        transform=axis.transAxes,
        va="top",
        ha="left",
        bbox={"facecolor": "white", "edgecolor": "0.7", "alpha": 0.88},
    )
    axis.legend(loc="upper right")

    figure.tight_layout()
    figure.savefig(OUTPUT, dpi=170, bbox_inches="tight")
    figure.savefig(MOTOR_STYLE_OUTPUT, dpi=170, bbox_inches="tight")
    figure.savefig(TURKISH_OUTPUT, dpi=170, bbox_inches="tight")

    knee_torque = []
    knee_speed = []
    knee_power = []
    for knee_column in [name for name in torque_data if name.endswith("_knee_torque_nm")]:
        knee_joint = knee_column[: -len(TORQUE_SUFFIX)]
        knee_speed_column = f"{knee_joint}{SPEED_SUFFIX}"
        knee_torque_nm = torque_data[knee_column]
        knee_speed_rpm = [value * RAD_PER_SEC_TO_RPM for value in speed_data[knee_speed_column]]
        knee_torque.extend(abs(value) for value in knee_torque_nm)
        knee_speed.extend(abs(value) for value in knee_speed_rpm)
        knee_power.extend(
            abs(torque_value * (speed_value / RAD_PER_SEC_TO_RPM))
            for torque_value, speed_value in zip(knee_torque_nm, knee_speed_rpm)
        )

    knee_max_torque = max(knee_torque)
    knee_max_speed = max(knee_speed)
    knee_max_power = max(knee_power)
    knee_p95_torque = percentile(knee_torque, 0.95)
    knee_p95_speed = percentile(knee_speed, 0.95)
    knee_rms_torque = rms(knee_torque)
    knee_recommended_stall = knee_max_torque * 1.40
    knee_recommended_free_speed = recommended_no_load_speed(
        knee_torque,
        knee_speed,
        knee_recommended_stall,
    ) * 1.10
    knee_curve_torque = [
        knee_recommended_stall * index / 100.0
        for index in range(101)
    ]
    knee_curve_speed = [
        knee_recommended_free_speed * (1.0 - torque / knee_recommended_stall)
        for torque in knee_curve_torque
    ]

    knee_figure, knee_axis = plt.subplots(figsize=(9.0, 7.0))
    knee_axis.scatter(
        knee_torque,
        knee_speed,
        s=7,
        alpha=0.16,
        linewidths=0,
        color="tab:blue",
        label="knee simülasyon noktaları",
    )
    knee_axis.plot(
        knee_curve_torque,
        knee_curve_speed,
        color="tab:red",
        linewidth=2.3,
        label="önerilen knee motor eğrisi",
    )
    knee_axis.fill_between(knee_curve_torque, knee_curve_speed, 0.0, color="tab:red", alpha=0.08)
    knee_axis.axvline(knee_max_torque, color="tab:red", linewidth=1.0, alpha=0.65)
    knee_axis.axhline(knee_max_speed, color="tab:red", linewidth=1.0, alpha=0.65)
    knee_axis.axvline(knee_p95_torque, color="tab:orange", linewidth=1.0, alpha=0.8, linestyle="--")
    knee_axis.axhline(knee_p95_speed, color="tab:orange", linewidth=1.0, alpha=0.8, linestyle="--", label="%95 talep")
    knee_axis.grid(True, alpha=0.3)
    knee_axis.set_xlim(left=0.0)
    knee_axis.set_ylim(bottom=0.0)
    knee_axis.set_xlabel("Tork [Nm]")
    knee_axis.set_ylabel("Hız [rpm]")
    knee_axis.set_title(
        "Knee motor seçimi, 1:1 direkt sürüş "
        f"({knee_max_torque:.2f} Nm, {knee_max_speed:.1f} rpm)"
    )
    knee_axis.text(
        0.04,
        0.96,
        "Knee öneri:\n"
        f"tepe tork >= {knee_recommended_stall:.1f} Nm\n"
        f"boşta hız >= {knee_recommended_free_speed:.0f} rpm\n"
        "sürekli tork >= 10-12 Nm\n"
        "tepe güç >= 200-250 W",
        transform=knee_axis.transAxes,
        va="top",
        ha="left",
        bbox={"facecolor": "white", "edgecolor": "0.7", "alpha": 0.88},
    )
    knee_axis.legend(loc="upper right")
    knee_figure.tight_layout()
    knee_figure.savefig(TURKISH_KNEE_OUTPUT, dpi=170, bbox_inches="tight")

    print(f"highest joint: {joint}")
    print(f"max abs speed: {max_abs_speed:.1f} rpm")
    print(f"max abs torque: {max_abs_torque:.2f} Nm")
    print(f"95% abs speed: {p95_speed:.1f} rpm")
    print(f"95% abs torque: {p95_torque:.2f} Nm")
    print(f"rms speed: {rms_speed:.1f} rpm")
    print(f"rms torque: {rms_torque:.2f} Nm")
    print(f"max mechanical power: {max_power:.1f} W")
    print(
        "at max torque: "
        f"{abs_torque_nm[max_torque_index]:.2f} Nm, "
        f"{abs_speed_rpm[max_torque_index]:.1f} rpm"
    )
    print(
        "at max speed: "
        f"{abs_torque_nm[max_speed_index]:.2f} Nm, "
        f"{abs_speed_rpm[max_speed_index]:.1f} rpm"
    )
    print(
        "at max power: "
        f"{abs_torque_nm[max_power_index]:.2f} Nm, "
        f"{abs_speed_rpm[max_power_index]:.1f} rpm"
    )
    print(f"recommended output stall torque: {recommended_stall_torque:.1f} Nm")
    print(f"recommended output no-load speed: {recommended_free_speed:.1f} rpm")
    print(f"saved plot: {OUTPUT}")
    print(f"saved motor-style plot: {MOTOR_STYLE_OUTPUT}")
    print(f"saved Turkish plot: {TURKISH_OUTPUT}")
    print(f"knee max abs speed: {knee_max_speed:.1f} rpm")
    print(f"knee max abs torque: {knee_max_torque:.2f} Nm")
    print(f"knee 95% abs speed: {knee_p95_speed:.1f} rpm")
    print(f"knee 95% abs torque: {knee_p95_torque:.2f} Nm")
    print(f"knee rms torque: {knee_rms_torque:.2f} Nm")
    print(f"knee max mechanical power: {knee_max_power:.1f} W")
    print(f"knee recommended output stall torque: {knee_recommended_stall:.1f} Nm")
    print(f"knee recommended output no-load speed: {knee_recommended_free_speed:.1f} rpm")
    print(f"saved Turkish knee plot: {TURKISH_KNEE_OUTPUT}")


if __name__ == "__main__":
    main()
