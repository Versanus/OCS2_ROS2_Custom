# OCS2 Quad Mini

This repository contains the current `quad_mini_real` workflow for:

- MuJoCo simulation
- OCS2 MPC / WBC control
- real-robot bridge
- RViz visualization
- sim-to-real command bridging

The main maintained path in this repo is now `quad_mini_real`.

## What Works Today

The project is set up around three practical use cases:

1. Full local simulation with MuJoCo, MPC, RViz, and GUI
2. Real robot stack on the Orin
3. Viewer-only RViz + GUI on a connected PC

The launcher interface was simplified. The old `RUN_ROLE` / `MPC_HOST` arguments are no longer part of the active workflow.

## Requirements

- Ubuntu 22.04
- Docker
- Docker Compose
- X11 access if you want RViz or GUI windows

Install Docker and verify:

```bash
docker --version
docker compose version
```

## Clone

```bash
git clone https://github.com/Versanus/OCS2_quad_mini.git
cd OCS2_quad_mini
```

## New PC Quickstart

On a fresh Ubuntu 22.04 machine:

```bash
sudo apt update
sudo apt install -y git x11-xserver-utils
```

Then install Docker and Docker Compose plugin, make sure Docker is running, and verify:

```bash
docker --version
docker compose version
docker info
```

Then clone and build:

```bash
git clone https://github.com/Versanus/OCS2_quad_mini.git
cd OCS2_quad_mini
./build.sh
```

After the build:

```bash
source ./source_ws.sh
./run.sh quad_mini_real sim estimated debug rviz gui
```

## GPU Rendering

Runtime launchers use the single `docker-compose.yml`, which is configured for NVIDIA GPU rendering by default.

If GPU startup fails with an `nvidia-container-cli` error, the host Docker NVIDIA runtime is not healthy yet. Verify GPU containers separately with `docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi`.

## Build

Build the full workspace with:

```bash
./build.sh
```

This builds the Docker image and the ROS 2 workspace inside it.

## Workspace Environment

To source the workspace in a normal terminal:

```bash
source ./source_ws.sh
```

This exports:

```bash
ROS_DOMAIN_ID=23
```

by default.

## Main Commands

### Full launcher

```bash
./run.sh <robot> <backend> <contact_source> <debug> <rviz> <gui> <rviz_source>
```

For the current project, the main robot is:

```bash
quad_mini_real
```

Arguments:

- `backend`: `sim` or `real`
- `contact_source`: `mujoco` or `estimated`
- `debug`: `debug` or `nodebug`
- `rviz`: `rviz` or `norviz`
- `gui`: `gui` or `nogui`
- `rviz_source`: `auto`, `sim`, or `hardware`

### Viewer-only launcher

```bash
./run_real_viewer.sh quad_mini_real
```

This opens:

- RViz
- hardware GUI
- debug shell

and waits for real-hardware topics. It does not launch MuJoCo or MPC.

## Common Workflows

### 1. Full local simulation

```bash
./run.sh quad_mini_real sim estimated debug rviz gui
```

This starts:

- bridge
- user command
- MPC
- debug terminal
- RViz
- GUI

### 2. Real robot stack on the Orin

Run this on the Orin:

```bash
./run.sh quad_mini_real real estimated debug norviz nogui
```

This keeps the Orin focused on the control stack and avoids opening viewer windows there.

### 3. Viewer on the connected PC

Run this on the connected PC:

```bash
./run_real_viewer.sh quad_mini_real
```

This waits for:

- `/htdw_joint_state`
- `/odom`

Then it launches moving-base RViz and the GUI.

### 4. Show real hardware in RViz while local sim is running

If you are running local sim but want RViz to visualize the real robot instead of MuJoCo:

```bash
./run.sh quad_mini_real sim estimated debug rviz gui hardware
```

The last `hardware` argument switches RViz to the hardware-side topics.

## Recommended Daily Flow

### Orin

```bash
./run.sh quad_mini_real real estimated debug norviz nogui
```

### Connected PC

```bash
./run_real_viewer.sh quad_mini_real
```

This is the clean split for real-robot use.

## sim2real Bridge

The current `sim2real.py` bridge:

- subscribes to `joint_control_data`
- publishes to `joint_cmd`

The output topic is parameterized, so it can be changed if your hardware side expects a different command topic.

Relevant file:

- `src/Quadruped-Control-OCS2-ROS2/hardware_interface/scripts/sim2real.py`

## Important Topics

Main topics in the current workflow:

- `/htdw_joint_state`
- `/htdw_joint_cmd`
- `/imu/data`
- `/odom`
- `/simulator_sensor_data`
- `/simulator_state_data`
- `/joint_control_data`

Useful checks:

```bash
ros2 topic list
ros2 topic hz /htdw_joint_state
ros2 topic hz /odom
ros2 topic echo /simulator_sensor_data
```

## Root Scripts

### Active scripts

- `build.sh`
- `run.sh`
- `run_real_viewer.sh`
- `source_ws.sh`
- `rebuild_quick.sh`
- `rebuild_user_command.sh`

### Optional / specialized scripts

- `mujoco_env.sh`
  Used by the launch and rebuild flow for MuJoCo library setup.

- `git-gp`
  Small local helper that does `git add -A`, commit, and push. Not part of runtime.

### Legacy / one-leg-only scripts

These are not part of the main `quad_mini_real` workflow:

- `rebuild_one_leg.sh`
- `run_one_leg.sh`
- `run_one_leg_rviz.sh`

They belong to:

- `src/one_leg_pinocchio_control/`

If you do not use the one-leg controller anymore, these are the main cleanup candidates.

## Reports

There are two LaTeX reports in the repo:

- `quad_mini_usage_report.tex`
  Short practical usage guide

- `quad_mini_detailed_report.tex`
  Longer technical report

## Notes

- `quad_mini_real` is the intended maintained path
- `run.sh` always runs MPC where the command is launched
- `run_real_viewer.sh` is the clean viewer-only command for the connected PC
