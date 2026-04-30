# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Deploy a Quad Mini Real ONNX policy in C MuJoCo with optional rollout dumps."""

from __future__ import annotations

import argparse
import atexit
import json
from pathlib import Path

from etils import epath
import mujoco
import mujoco.viewer as viewer
import numpy as np
import onnxruntime as rt

from mujoco_playground._src.locomotion.quad_mini_real import joystick
from mujoco_playground._src.locomotion.quad_mini_real import (
    quad_mini_real_constants as consts,
)
from mujoco_playground._src.locomotion.quad_mini_real.base import get_assets
from mujoco_playground.experimental.sim2sim.gamepad_reader import Gamepad

_HERE = epath.Path(__file__).parent
_ONNX_DIR = _HERE / "onnx"


def _parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--terrain",
      choices=("rough", "flat"),
      default="rough",
      help="Which MuJoCo XML terrain to load.",
  )
  parser.add_argument(
      "--policy",
      default=None,
      help=(
          "Optional ONNX policy path. Defaults to quad_mini_rough_policy.onnx "
          "for both terrains unless overridden."
      ),
  )
  parser.add_argument(
      "--command",
      type=float,
      nargs=3,
      metavar=("VX", "VY", "WZ"),
      default=None,
      help="Fixed command in policy units. If unset, reads from gamepad.",
  )
  parser.add_argument(
      "--dump",
      default=None,
      help="Optional .npz path to save policy-step rollout arrays.",
  )
  parser.add_argument(
      "--max-policy-steps",
      type=int,
      default=0,
      help="Optional limit on logged policy steps. 0 means unlimited.",
  )
  parser.add_argument(
      "--clip-actions",
      action="store_true",
      help="Clip policy actions to [-1, 1] before sending q_des to MuJoCo.",
  )
  parser.add_argument(
      "--headless",
      action="store_true",
      help="Run without the viewer and exit after collecting the requested dump.",
  )
  return parser.parse_args()


_ARGS = _parse_args()
_LOGGER: "RolloutLogger | None" = None


def _terrain_xml(terrain: str) -> epath.Path:
  if terrain == "flat":
    return consts.FLAT_TERRAIN_XML
  return consts.ROUGH_TERRAIN_XML


class RolloutLogger:
  """Collects policy-step rollout arrays and writes them to disk on exit."""

  def __init__(
      self,
      dump_path: str | None,
      max_policy_steps: int,
      metadata: dict[str, object],
  ):
    self._dump_path = Path(dump_path).expanduser().resolve() if dump_path else None
    self._max_policy_steps = max_policy_steps
    self._metadata = metadata
    self._records: list[dict[str, np.ndarray | float]] = []
    self._pending_index: int | None = None
    self._saved = False

  def enabled(self) -> bool:
    return self._dump_path is not None

  def should_log(self) -> bool:
    return self.enabled() and (
        self._max_policy_steps <= 0 or len(self._records) < self._max_policy_steps
    )

  def begin_step(self, model: mujoco.MjModel, data: mujoco.MjData, record: dict[str, np.ndarray | float]) -> None:
    if not self.should_log():
      return

    if self._pending_index is not None:
      previous = self._records[self._pending_index]
      previous["next_qpos"] = np.array(data.qpos, dtype=np.float64)
      previous["next_qvel"] = np.array(data.qvel, dtype=np.float64)
      previous["next_sensordata"] = np.array(data.sensordata, dtype=np.float64)
      previous["next_time"] = float(data.time)

    record = dict(record)
    record["policy_step"] = float(len(self._records))
    record["qpos"] = np.array(data.qpos, dtype=np.float64)
    record["qvel"] = np.array(data.qvel, dtype=np.float64)
    record["sensordata"] = np.array(data.sensordata, dtype=np.float64)
    record["time"] = float(data.time)
    self._records.append(record)
    self._pending_index = len(self._records) - 1

  def save(self) -> None:
    if self._saved or not self.enabled():
      return
    self._saved = True
    if not self._records:
      print(f"No rollout records to save for {self._dump_path}")
      return

    self._dump_path.parent.mkdir(parents=True, exist_ok=True)
    keys: set[str] = set()
    for record in self._records:
      keys.update(record.keys())

    arrays: dict[str, np.ndarray] = {}
    for key in sorted(keys):
      values = [record.get(key) for record in self._records]
      if any(value is None for value in values):
        continue
      first = values[0]
      if isinstance(first, np.ndarray):
        arrays[key] = np.stack(values, axis=0)
      else:
        arrays[key] = np.asarray(values, dtype=np.float64)

    np.savez(self._dump_path, **arrays)
    metadata_path = self._dump_path.with_suffix(".json")
    metadata_path.write_text(json.dumps(self._metadata, indent=2), encoding="utf-8")
    print(f"Saved rollout dump to {self._dump_path}")
    print(f"Saved rollout metadata to {metadata_path}")


class OnnxController:
  """ONNX controller for the Quad Mini Real robot."""

  def __init__(
      self,
      policy_path: str,
      default_angles: np.ndarray,
      n_substeps: int,
      logger: RolloutLogger,
      action_scale: float = 0.5,
      vel_scale_x: float = 1.5,
      vel_scale_y: float = 0.8,
      vel_scale_rot: float = 2 * np.pi,
      fixed_command: np.ndarray | None = None,
      clip_actions: bool = False,
  ):
    self._output_names = ["continuous_actions"]
    self._policy = rt.InferenceSession(
        policy_path, providers=["CPUExecutionProvider"]
    )

    self._action_scale = action_scale
    self._default_angles = default_angles.astype(np.float32)
    self._last_action = np.zeros_like(default_angles, dtype=np.float32)
    self._logger = logger
    self._clip_actions = clip_actions

    self._counter = 0
    self._n_substeps = n_substeps
    self._fixed_command = (
        np.asarray(fixed_command, dtype=np.float32)
        if fixed_command is not None
        else None
    )
    self._joystick = None
    if self._fixed_command is None:
      self._joystick = Gamepad(
          vel_scale_x=vel_scale_x,
          vel_scale_y=vel_scale_y,
          vel_scale_rot=vel_scale_rot,
      )

  def _command(self) -> np.ndarray:
    if self._fixed_command is not None:
      return self._fixed_command.copy()
    return self._joystick.get_command().astype(np.float32)

  def get_obs(self, model, data) -> dict[str, np.ndarray]:
    linvel = np.array(data.sensor(consts.LOCAL_LINVEL_SENSOR).data, dtype=np.float32)
    gyro = np.array(data.sensor(consts.GYRO_SENSOR).data, dtype=np.float32)
    imu_xmat = data.site_xmat[model.site(consts.IMU_SITE).id].reshape(3, 3)
    gravity = (imu_xmat.T @ np.array([0.0, 0.0, -1.0], dtype=np.float64)).astype(
        np.float32
    )
    joint_angles = (np.array(data.qpos[7:], dtype=np.float32) - self._default_angles)
    joint_velocities = np.array(data.qvel[6:], dtype=np.float32)
    previous_action = self._last_action.copy()
    command = self._command()
    obs = np.hstack([
        linvel,
        gyro,
        gravity,
        joint_angles,
        joint_velocities,
        previous_action,
        command,
    ]).astype(np.float32)
    return {
        "obs": obs,
        "base_lin_vel": linvel,
        "base_ang_vel": gyro,
        "projected_gravity": gravity,
        "joint_pos": joint_angles,
        "joint_vel": joint_velocities,
        "previous_action": previous_action,
        "command": command,
        "base_quat": np.array(data.qpos[3:7], dtype=np.float32),
    }

  def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    self._counter += 1
    if self._counter % self._n_substeps != 0:
      return

    obs_snapshot = self.get_obs(model, data)
    onnx_input = obs_snapshot["obs"].reshape(1, -1)
    action_raw = self._policy.run(self._output_names, {"obs": onnx_input})[0][0]
    action_raw = np.asarray(action_raw, dtype=np.float32)
    action_clipped = np.clip(action_raw, -1.0, 1.0).astype(np.float32)
    action_for_ctrl = action_clipped if self._clip_actions else action_raw
    q_des = action_for_ctrl * self._action_scale + self._default_angles

    self._logger.begin_step(
        model,
        data,
        {
            "base_quat": obs_snapshot["base_quat"],
            "base_lin_vel": obs_snapshot["base_lin_vel"],
            "base_ang_vel": obs_snapshot["base_ang_vel"],
            "projected_gravity": obs_snapshot["projected_gravity"],
            "joint_pos": obs_snapshot["joint_pos"],
            "joint_vel": obs_snapshot["joint_vel"],
            "command": obs_snapshot["command"],
            "previous_action": obs_snapshot["previous_action"],
            "obs_raw": obs_snapshot["obs"],
            "onnx_input": obs_snapshot["obs"],
            "onnx_output_raw": action_raw,
            "action_clipped": action_clipped,
            "q_des": q_des,
            "ctrl": q_des.copy(),
        },
    )

    self._last_action = action_raw.copy()
    data.ctrl[:] = q_des


def load_callback(model=None, data=None):
  del model, data
  mujoco.set_mjcb_control(None)

  xml_path = _terrain_xml(_ARGS.terrain)
  policy_path = (
      Path(_ARGS.policy).expanduser().resolve()
      if _ARGS.policy
      else (_ONNX_DIR / "quad_mini_rough_policy.onnx").resolve()
  )
  model = mujoco.MjModel.from_xml_path(
      xml_path.as_posix(),
      assets=get_assets(),
  )
  data = mujoco.MjData(model)

  mujoco.mj_resetDataKeyframe(model, data, 0)

  config = joystick.default_config()
  config.ccd_iterations = 100

  ctrl_dt = config.ctrl_dt
  sim_dt = config.sim_dt
  n_substeps = int(round(ctrl_dt / sim_dt))
  model.opt.timestep = sim_dt
  model.opt.ccd_iterations = config.ccd_iterations
  model.dof_damping[6:] = config.Kd
  model.actuator_gainprm[:, 0] = config.Kp
  model.actuator_biasprm[:, 1] = -config.Kp

  metadata = {
      "terrain": _ARGS.terrain,
      "xml_path": xml_path.as_posix(),
      "policy_path": policy_path.as_posix(),
      "model_timestep": float(model.opt.timestep),
      "policy_dt": float(ctrl_dt),
      "action_repeat": int(n_substeps),
      "action_scale": float(config.action_scale),
      "Kp": float(config.Kp),
      "Kd": float(config.Kd),
      "default_angles": np.array(model.keyframe("home").qpos[7:], dtype=np.float64).tolist(),
      "joint_names_qpos_order": [
          model.joint(i).name for i in range(1, model.njnt)
      ],
      "actuator_names": [model.actuator(i).name for i in range(model.nu)],
      "sensor_names": [model.sensor(i).name for i in range(model.nsensor)],
      "fixed_command": _ARGS.command,
      "clip_actions": bool(_ARGS.clip_actions),
  }
  logger = RolloutLogger(_ARGS.dump, _ARGS.max_policy_steps, metadata)
  atexit.register(logger.save)
  global _LOGGER
  _LOGGER = logger

  for i in range(model.nu):
    print(i, model.actuator(i).name)
    print("gain", model.actuator_gainprm[i])
    print("bias", model.actuator_biasprm[i])
    print("forcerange", model.actuator_forcerange[i])

  if _ARGS.command is not None:
    print(f"Using fixed command {_ARGS.command} in place of the gamepad.")
  print(f"Terrain XML: {xml_path}")
  print(f"Policy path: {policy_path}")
  if logger.enabled():
    print(f"Rollout dump will be written to {logger._dump_path}")

  policy = OnnxController(
      policy_path=policy_path.as_posix(),
      default_angles=np.array(model.keyframe("home").qpos[7:]),
      n_substeps=n_substeps,
      logger=logger,
      action_scale=config.action_scale,
      vel_scale_x=config.command_config.a[0],
      vel_scale_y=config.command_config.a[1],
      vel_scale_rot=config.command_config.a[2],
      fixed_command=None if _ARGS.command is None else np.asarray(_ARGS.command, dtype=np.float32),
      clip_actions=_ARGS.clip_actions,
  )

  mujoco.set_mjcb_control(policy.get_control)

  return model, data


if __name__ == "__main__":
  if _ARGS.headless:
    if _ARGS.max_policy_steps <= 0:
      raise SystemExit("--headless requires --max-policy-steps > 0.")
    model, data = load_callback()
    while _LOGGER is not None and len(_LOGGER._records) < _ARGS.max_policy_steps:
      mujoco.mj_step(model, data)
    if _LOGGER is not None:
      _LOGGER.save()
  else:
    viewer.launch(loader=load_callback)
