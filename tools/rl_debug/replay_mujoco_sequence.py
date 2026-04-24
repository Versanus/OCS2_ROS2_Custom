#!/usr/bin/env python3
"""Replay a saved control sequence in MuJoCo and compare the state trajectory."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from replay_mujoco_step import apply_rl_runtime_overrides


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", required=True, help="Path to the MuJoCo XML/MJCF file.")
    parser.add_argument("--sample-npz", required=True, help="Path to a rollout .npz file.")
    parser.add_argument("--rl-config", help="Optional rl.info file for bridge-equivalent runtime overrides.")
    parser.add_argument("--qpos-key", default="qpos", help="NPZ key for qpos trajectory.")
    parser.add_argument("--qvel-key", default="qvel", help="NPZ key for qvel trajectory.")
    parser.add_argument("--ctrl-key", default="ctrl", help="NPZ key for control trajectory.")
    parser.add_argument("--time-key", default="time", help="Optional NPZ key for sample timestamps.")
    parser.add_argument("--start-index", type=int, default=0, help="First trajectory index to replay.")
    parser.add_argument("--num-steps", type=int, default=0, help="How many steps to replay. 0 means as many as possible.")
    parser.add_argument("--physics-steps-per-sample", type=int, default=0, help="Override how many mj_step calls to apply per sample. 0 infers from time/model timestep when possible.")
    parser.add_argument("--qpos-threshold", type=float, default=1e-3, help="Report first step exceeding this qpos max-abs diff.")
    parser.add_argument("--qvel-threshold", type=float, default=1e-2, help="Report first step exceeding this qvel max-abs diff.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    xml_path = Path(args.xml).resolve()
    sample_path = Path(args.sample_npz).resolve()
    rl_config_path = Path(args.rl_config).resolve() if args.rl_config else None

    rollout = np.load(sample_path, allow_pickle=False)
    qpos = np.asarray(rollout[args.qpos_key], dtype=np.float64)
    qvel = np.asarray(rollout[args.qvel_key], dtype=np.float64)
    ctrl = np.asarray(rollout[args.ctrl_key], dtype=np.float64)
    time = np.asarray(rollout[args.time_key], dtype=np.float64) if args.time_key in rollout else None

    if qpos.ndim != 2 or qvel.ndim != 2 or ctrl.ndim != 2:
        raise ValueError("qpos, qvel, and ctrl arrays must all be 2-D trajectories.")
    if not (qpos.shape[0] == qvel.shape[0] == ctrl.shape[0]):
        raise ValueError("qpos, qvel, and ctrl must have the same trajectory length.")
    if qpos.shape[0] < 2:
        raise ValueError("Need at least 2 trajectory samples for sequence replay.")

    start = max(0, args.start_index)
    last_usable = qpos.shape[0] - 1
    if start >= last_usable:
        raise ValueError(f"start-index {start} must be smaller than {last_usable}.")

    if args.num_steps > 0:
        steps = min(args.num_steps, last_usable - start)
    else:
        steps = last_usable - start

    model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
    runtime_overrides: dict[str, Any] | None = None
    if rl_config_path is not None:
        runtime_overrides = apply_rl_runtime_overrides(model, rl_config_path)
    data = mujoco.MjData(model)

    if qpos.shape[1] != model.nq:
        raise ValueError(f"qpos width mismatch: trajectory has {qpos.shape[1]}, model expects {model.nq}.")
    if qvel.shape[1] != model.nv:
        raise ValueError(f"qvel width mismatch: trajectory has {qvel.shape[1]}, model expects {model.nv}.")
    if ctrl.shape[1] != model.nu:
        raise ValueError(f"ctrl width mismatch: trajectory has {ctrl.shape[1]}, model expects {model.nu}.")

    data.qpos[:] = qpos[start]
    data.qvel[:] = qvel[start]
    mujoco.mj_forward(model, data)

    if args.physics_steps_per_sample > 0:
        physics_steps = args.physics_steps_per_sample
    elif time is not None and time.ndim == 1 and time.shape[0] == qpos.shape[0] and qpos.shape[0] >= 2:
        sample_dt = float(np.median(np.diff(time)))
        physics_steps = max(1, int(round(sample_dt / model.opt.timestep)))
    else:
        physics_steps = 1

    qpos_diffs: list[float] = []
    qvel_diffs: list[float] = []
    first_bad_qpos: tuple[int, float] | None = None
    first_bad_qvel: tuple[int, float] | None = None

    for step_offset in range(steps):
        idx = start + step_offset
        data.ctrl[:] = ctrl[idx]
        mujoco.mj_forward(model, data)
        for _ in range(physics_steps):
            mujoco.mj_step(model, data)

        qpos_diff = float(np.max(np.abs(np.asarray(data.qpos) - qpos[idx + 1])))
        qvel_diff = float(np.max(np.abs(np.asarray(data.qvel) - qvel[idx + 1])))
        qpos_diffs.append(qpos_diff)
        qvel_diffs.append(qvel_diff)

        if first_bad_qpos is None and qpos_diff > args.qpos_threshold:
            first_bad_qpos = (idx, qpos_diff)
        if first_bad_qvel is None and qvel_diff > args.qvel_threshold:
            first_bad_qvel = (idx, qvel_diff)

    print(f"xml                 : {xml_path}")
    if rl_config_path is not None:
        print(f"rl_config           : {rl_config_path}")
        print(
            "runtime             : "
            f"timestep={runtime_overrides['timestep']:.9f} "
            f"base_kp={runtime_overrides['base_kp']:.6f} "
            f"base_kd={runtime_overrides['base_kd']:.6f} "
            f"direct_position={runtime_overrides['direct_position']}"
        )
    print(f"sample              : {sample_path}")
    print(f"trajectory_length   : {qpos.shape[0]}")
    print(f"start_index         : {start}")
    print(f"replayed_steps      : {steps}")
    print(f"physics_steps/sample: {physics_steps}")
    print(f"qpos_max_diff       : {max(qpos_diffs):.9e}")
    print(f"qpos_median_diff    : {float(np.median(qpos_diffs)):.9e}")
    print(f"qvel_max_diff       : {max(qvel_diffs):.9e}")
    print(f"qvel_median_diff    : {float(np.median(qvel_diffs)):.9e}")
    print(
        "first_qpos_threshold: "
        + (f"idx={first_bad_qpos[0]} diff={first_bad_qpos[1]:.9e}" if first_bad_qpos else "none")
    )
    print(
        "first_qvel_threshold: "
        + (f"idx={first_bad_qvel[0]} diff={first_bad_qvel[1]:.9e}" if first_bad_qvel else "none")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
