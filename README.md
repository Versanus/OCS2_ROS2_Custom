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
- `git`
- Docker Engine with the Docker Compose plugin
- X11 access if you want MuJoCo, RViz, or GUI windows
- NVIDIA driver + NVIDIA Container Toolkit if you want GPU MuJoCo rendering

This repo is built around the normal apt-based Docker Engine on Ubuntu. Do not use the Snap Docker package on a new machine.

## New PC Quickstart

Use these steps on a fresh Ubuntu 22.04 machine.

### 1. Install base packages

```bash
sudo apt update
sudo apt install -y git curl ca-certificates x11-xserver-utils
```

### 2. Install Docker Engine

Install Docker Engine and the Compose plugin using Docker's official Ubuntu instructions:

- Docker Engine on Ubuntu: https://docs.docker.com/engine/install/ubuntu/
- Docker Compose plugin on Linux: https://docs.docker.com/compose/install/linux/

The standard apt-based install is:

```bash
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo tee /etc/apt/keyrings/docker.asc >/dev/null
sudo chmod a+r /etc/apt/keyrings/docker.asc

cat <<EOF | sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

Log out and log back in after adding yourself to the `docker` group.

### 3. Install GPU container support

MuJoCo in this repo is configured for NVIDIA GPU rendering by default.

First verify the host driver:

```bash
nvidia-smi
```

If that does not work, fix the NVIDIA driver first.

Then install NVIDIA Container Toolkit using the official guide:

- NVIDIA Container Toolkit install guide: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

The apt-based install is:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

sudo apt update
sudo apt install -y nvidia-container-toolkit
```

Then configure Docker and restart the daemon:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 4. Validate the host setup

Before building the repo, make sure Docker and GPU containers work:

```bash
docker --version
docker compose version
docker info
docker run --rm hello-world
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

If the last command fails, fix the host Docker/NVIDIA runtime before building this workspace.

### 5. Clone the repo

```bash
git clone https://github.com/Versanus/OCS2_quad_mini.git
cd OCS2_quad_mini
```

### 6. Build the workspace

```bash
./build.sh
```

`./build.sh` will:

- clone the missing external dependencies under `src/Quadruped-Control-OCS2-ROS2`
- build `qpOASES`
- build the Docker image
- build the ROS 2 workspace inside the container

### 7. Source the workspace and run sim

```bash
source ./source_ws.sh
./run.sh quad_mini_real sim estimated debug rviz gui
```

## GPU Rendering

Runtime launchers use the single `docker-compose.yml`, which is configured for NVIDIA GPU rendering by default.

If GPU startup fails with an `nvidia-container-cli` error, the host Docker NVIDIA runtime is not healthy yet. Verify GPU containers separately with:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

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
