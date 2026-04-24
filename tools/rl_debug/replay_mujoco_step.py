#!/usr/bin/env python3
"""Replay a saved MuJoCo state/control sample and compare the next state."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


COMMAND_JOINT_NAMES = (
    "LF_HAA", "LF_HFE", "LF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xml", required=True, help="Path to the MuJoCo XML/MJCF file.")
    parser.add_argument("--rl-config", help="Optional rl.info file. Applies the same runtime overrides as the bridge.")
    parser.add_argument("--sample-json", help="Path to a simulator control_step_XXXXXX.json sample.")
    parser.add_argument("--sample-npz", help="Path to a .npz file containing replay arrays.")
    parser.add_argument("--index", type=int, default=0, help="Step index when using --sample-npz.")
    parser.add_argument("--qpos-key", default="qpos", help="NPZ key for qpos when using --sample-npz.")
    parser.add_argument("--qvel-key", default="qvel", help="NPZ key for qvel when using --sample-npz.")
    parser.add_argument("--ctrl-key", default="ctrl", help="NPZ key for ctrl when using --sample-npz.")
    parser.add_argument("--next-qpos-key", default="next_qpos", help="NPZ key for next qpos when using --sample-npz.")
    parser.add_argument("--next-qvel-key", default="next_qvel", help="NPZ key for next qvel when using --sample-npz.")
    parser.add_argument("--steps", type=int, help="Override the number of physics steps to replay.")
    parser.add_argument("--top-k", type=int, default=10, help="How many largest state diffs to print.")
    args = parser.parse_args()

    if bool(args.sample_json) == bool(args.sample_npz):
        parser.error("Provide exactly one of --sample-json or --sample-npz.")
    return args


def top_differences(lhs: np.ndarray, rhs: np.ndarray, top_k: int) -> list[str]:
    diff = np.abs(lhs - rhs)
    order = np.argsort(diff)[::-1][:top_k]
    lines: list[str] = []
    for idx in order:
        lines.append(
            f"  idx={int(idx):2d} ref={float(rhs[idx]): .9f} replay={float(lhs[idx]): .9f} abs_diff={float(diff[idx]):.9e}"
        )
    return lines


def _load_vector(container: Any, key: str, *, required: bool = True) -> np.ndarray | None:
    if key not in container:
        if required:
            raise KeyError(f"Missing required key '{key}'.")
        return None
    return np.asarray(container[key], dtype=np.float64)


def parse_info_scalars(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split(";", 1)[0].strip()
        if not line or line.endswith("{") or line == "}" or line.startswith("[") or line.startswith("("):
            continue
        parts = line.split(None, 1)
        if len(parts) == 2:
            values[parts[0]] = parts[1].strip()
    return values


def _parse_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def apply_rl_runtime_overrides(model: mujoco.MjModel, rl_config: Path) -> dict[str, Any]:
    scalars = parse_info_scalars(rl_config)

    timestep = float(scalars.get("mujocoTimestep", model.opt.timestep))
    base_kp = float(scalars.get("mujocoBaseKp", 0.0))
    base_kd = float(scalars.get("mujocoBaseKd", 0.0))
    direct_position = _parse_bool(scalars.get("mujocoDirectPositionControl", "false"))

    model.opt.timestep = timestep

    if direct_position:
        for joint_name in COMMAND_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
            if joint_id < 0 or actuator_id < 0:
                raise ValueError(f"Missing joint/actuator '{joint_name}' while applying rl.info overrides.")
            dof_adr = model.jnt_dofadr[joint_id]
            model.actuator_gaintype[actuator_id] = mujoco.mjtGain.mjGAIN_FIXED
            model.actuator_biastype[actuator_id] = mujoco.mjtBias.mjBIAS_AFFINE
            model.actuator_gainprm[actuator_id, 0] = base_kp
            model.actuator_biasprm[actuator_id, 0] = 0.0
            model.actuator_biasprm[actuator_id, 1] = -base_kp
            model.actuator_biasprm[actuator_id, 2] = 0.0
            model.dof_damping[dof_adr] = base_kd

    return {
        "timestep": timestep,
        "base_kp": base_kp,
        "base_kd": base_kd,
        "direct_position": direct_position,
    }


def load_json_sample(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    sample: dict[str, Any] = {
        "source": path.as_posix(),
        "qpos": _load_vector(data, "pre_qpos"),
        "qvel": _load_vector(data, "pre_qvel"),
        "ctrl": _load_vector(data, "post_ctrl"),
        "expected_qpos": _load_vector(data, "post_qpos", required=False),
        "expected_qvel": _load_vector(data, "post_qvel", required=False),
        "pre_time": float(data["pre_time"]) if "pre_time" in data else None,
        "post_time": float(data["post_time"]) if "post_time" in data else None,
    }
    return sample


def _select_row(array: np.ndarray, index: int, key: str) -> np.ndarray:
    if array.ndim == 1:
        return array.astype(np.float64)
    if array.ndim != 2:
        raise ValueError(f"NPZ key '{key}' must be 1-D or 2-D, got shape {array.shape}.")
    if not (0 <= index < array.shape[0]):
        raise IndexError(f"Index {index} is out of bounds for NPZ key '{key}' with shape {array.shape}.")
    return array[index].astype(np.float64)


def load_npz_sample(path: Path, args: argparse.Namespace) -> dict[str, Any]:
    archive = np.load(path, allow_pickle=False)
    sample: dict[str, Any] = {
        "source": f"{path.as_posix()}[{args.index}]",
        "qpos": _select_row(_load_vector(archive, args.qpos_key), args.index, args.qpos_key),
        "qvel": _select_row(_load_vector(archive, args.qvel_key), args.index, args.qvel_key),
        "ctrl": _select_row(_load_vector(archive, args.ctrl_key), args.index, args.ctrl_key),
        "expected_qpos": None,
        "expected_qvel": None,
        "pre_time": None,
        "post_time": None,
    }
    if args.next_qpos_key in archive:
        sample["expected_qpos"] = _select_row(_load_vector(archive, args.next_qpos_key), args.index, args.next_qpos_key)
    if args.next_qvel_key in archive:
        sample["expected_qvel"] = _select_row(_load_vector(archive, args.next_qvel_key), args.index, args.next_qvel_key)
    return sample


def main() -> int:
    args = parse_args()
    xml_path = Path(args.xml).resolve()
    rl_config_path = Path(args.rl_config).resolve() if args.rl_config else None
    if args.sample_json:
        sample = load_json_sample(Path(args.sample_json).resolve())
    else:
        sample = load_npz_sample(Path(args.sample_npz).resolve(), args)

    model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
    runtime_overrides = None
    if rl_config_path is not None:
        runtime_overrides = apply_rl_runtime_overrides(model, rl_config_path)
    data = mujoco.MjData(model)

    qpos = np.asarray(sample["qpos"], dtype=np.float64)
    qvel = np.asarray(sample["qvel"], dtype=np.float64)
    ctrl = np.asarray(sample["ctrl"], dtype=np.float64)
    expected_qpos = sample["expected_qpos"]
    expected_qvel = sample["expected_qvel"]

    if qpos.shape[0] != model.nq:
        raise ValueError(f"qpos length mismatch: sample has {qpos.shape[0]}, model expects {model.nq}.")
    if qvel.shape[0] != model.nv:
        raise ValueError(f"qvel length mismatch: sample has {qvel.shape[0]}, model expects {model.nv}.")
    if ctrl.shape[0] != model.nu:
        raise ValueError(f"ctrl length mismatch: sample has {ctrl.shape[0]}, model expects {model.nu}.")

    data.qpos[:] = qpos
    data.qvel[:] = qvel
    data.ctrl[:] = ctrl
    mujoco.mj_forward(model, data)

    if args.steps is not None:
        steps = max(1, args.steps)
    else:
        pre_time = sample.get("pre_time")
        post_time = sample.get("post_time")
        if pre_time is not None and post_time is not None and post_time > pre_time:
            steps = max(1, int(round((post_time - pre_time) / model.opt.timestep)))
        else:
            steps = 1

    for _ in range(steps):
        mujoco.mj_step(model, data)

    replay_qpos = np.array(data.qpos, dtype=np.float64)
    replay_qvel = np.array(data.qvel, dtype=np.float64)

    print(f"xml            : {xml_path}")
    if rl_config_path is not None:
        print(f"rl_config      : {rl_config_path}")
        print(
            "runtime        : "
            f"timestep={runtime_overrides['timestep']:.9f} "
            f"base_kp={runtime_overrides['base_kp']:.6f} "
            f"base_kd={runtime_overrides['base_kd']:.6f} "
            f"direct_position={runtime_overrides['direct_position']}"
        )
    print(f"sample         : {sample['source']}")
    print(f"timestep       : {float(model.opt.timestep):.9f}")
    print(f"physics_steps  : {steps}")
    if sample.get("pre_time") is not None:
        print(f"pre_time       : {sample['pre_time']:.9f}")
    if sample.get("post_time") is not None:
        print(f"post_time      : {sample['post_time']:.9f}")

    if expected_qpos is not None:
        expected_qpos = np.asarray(expected_qpos, dtype=np.float64)
        qpos_max_abs_diff = float(np.max(np.abs(replay_qpos - expected_qpos)))
        print(f"qpos_max_diff  : {qpos_max_abs_diff:.9e}")
        print("qpos_top_diffs :")
        for line in top_differences(replay_qpos, expected_qpos, args.top_k):
            print(line)
    else:
        print("expected qpos  : missing")

    if expected_qvel is not None:
        expected_qvel = np.asarray(expected_qvel, dtype=np.float64)
        qvel_max_abs_diff = float(np.max(np.abs(replay_qvel - expected_qvel)))
        print(f"qvel_max_diff  : {qvel_max_abs_diff:.9e}")
        print("qvel_top_diffs :")
        for line in top_differences(replay_qvel, expected_qvel, args.top_k):
            print(line)
    else:
        print("expected qvel  : missing")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
