#!/usr/bin/env python3
"""Replay a dumped RL controller observation through ONNX Runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict

import numpy as np
import onnxruntime as ort


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rl-config", required=True, help="Path to rl.info")
    parser.add_argument("--dump", required=True, help="Path to controller policy_step_XXXXXX.json")
    parser.add_argument("--top-k", type=int, default=10, help="How many largest action diffs to print")
    return parser.parse_args()


def parse_info_scalars(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split(";", 1)[0].strip()
        if not line or line.endswith("{") or line == "}" or line.startswith("[") or line.startswith("("):
            continue
        parts = line.split(None, 1)
        if len(parts) == 2:
            values[parts[0]] = parts[1].strip()
    return values


def top_differences(lhs: np.ndarray, rhs: np.ndarray, top_k: int) -> str:
    diff = np.abs(lhs - rhs)
    order = np.argsort(diff)[::-1][:top_k]
    lines = []
    for idx in order:
        lines.append(
            f"  idx={int(idx):2d} ref={float(rhs[idx]): .8f} replay={float(lhs[idx]): .8f} abs_diff={float(diff[idx]):.8e}"
        )
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    rl_config = Path(args.rl_config).resolve()
    dump_path = Path(args.dump).resolve()

    scalars = parse_info_scalars(rl_config)
    model_path = (rl_config.parent / scalars["onnxModelPath"]).resolve()
    input_name = scalars.get("onnxInputName", "obs")
    output_name = scalars.get("onnxOutputName", "continuous_actions")

    dump = json.loads(dump_path.read_text(encoding="utf-8"))
    onnx_input = np.asarray(dump["onnx_input"], dtype=np.float32).reshape(1, -1)
    expected_output = np.asarray(dump.get("onnx_output_raw", []), dtype=np.float32)

    session = ort.InferenceSession(model_path.as_posix(), providers=["CPUExecutionProvider"])
    replay_output = session.run([output_name], {input_name: onnx_input})[0][0].astype(np.float32)

    max_abs_diff = float(np.max(np.abs(replay_output - expected_output))) if expected_output.size else float("nan")

    print(f"rl_config      : {rl_config}")
    print(f"dump           : {dump_path}")
    print(f"model          : {model_path}")
    print(f"input_name     : {input_name}")
    print(f"output_name    : {output_name}")
    print(f"obs_dim        : {onnx_input.shape[1]}")
    print(f"action_dim     : {replay_output.shape[0]}")
    if expected_output.size:
        print(f"max_abs_diff   : {max_abs_diff:.8e}")
        print("largest_diffs  :")
        print(top_differences(replay_output, expected_output, args.top_k))
    else:
        print("expected output missing from dump file")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
