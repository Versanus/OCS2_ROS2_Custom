# Quadruped-Control-OCS2-ROS2

## Introduction

This project implements a **quadruped robot simulation and NMPC control framework** using **ROS 2**, **OCS2**, and **MuJoCo**.

The system integrates:

* user command interface
* nonlinear model predictive control (NMPC)
* quadruped robot physics simulation
* ROS 2 communication and launch infrastructure

The project is based on the open-source work from:

* https://github.com/leggedrobotics/ocs2
* https://github.com/qiayuanl/legged_control
* https://github.com/zhengxiang94/ocs2_ros2

---

## Simulation Overview

![structure](.image/structure.jpg)

![mujoco](.image/mujoco_simulation_image.png)

---

# Quick Start (Recommended)

The easiest way to run the project is using Docker.

### Requirements

* Ubuntu 22.04
* Docker Engine
* Docker Compose plugin (`docker compose`)
* X11 (for MuJoCo GUI)

---

## Clone the Repository

```bash
git clone https://github.com/Versanus/OCS2_quad_mini.git
cd OCS2_quad_mini
```

---

## Build the Environment

```bash
./build.sh
```

This script will:

* build the Docker image
* build the ROS2 workspace
* configure MuJoCo and qpOASES dependencies

---

## Run the Simulation

```bash
./run.sh
```

This launches the quadruped simulation using MuJoCo.

---

# Manual Docker Workflow (Optional)

If you prefer to run Docker commands manually:

### Enable X11 access

```bash
xhost +local:docker
```

---

### Build Docker image

```bash
docker compose build
```

---

### Enter the container

```bash
docker compose run --rm quad_ocs2 bash
```

---

### Build the workspace

```bash
colcon build \
  --packages-up-to motion_control mujoco_simulator user_command launch_simulation \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

---

### Run simulation

```bash
./run.sh
```

---

# Simulation Controls

The `user_command` interface allows controlling the robot during simulation.

Examples:

```
gait:trot
goal:1 1 0 0
```

Where:

```
goal: x y z yaw
```

Example:

```
goal:1 1 0 0
```

moves the robot to position `(x=1, y=1)` with yaw `0`.

---

# Project Structure

```
OCS2_quad_mini
│
├── docker/                  Docker environment
├── src/                     ROS2 packages
├── tools/                   helper utilities
│
├── build.sh                 build the workspace
├── run.sh                   launch simulation
├── rebuild_quick.sh         fast rebuild for development
├── docker-compose.yml
│
└── README.md
```

---

# Dependencies

This project uses:

* ROS 2 Humble
* MuJoCo
* OCS2
* qpOASES
* grid_map
* OctoMap

All dependencies are automatically handled inside the Docker environment.

---

# Development

For quick package rebuilds during development:

```bash
./rebuild_quick.sh
```

This rebuilds only the necessary packages instead of the entire workspace.

---

# Original Project References

This repository builds upon the following open-source work:

* https://github.com/HexiangZhou/Quadruped-Control-OCS2-ROS2
* https://github.com/zhengxiang94/ocs2_ros2
* https://github.com/leggedrobotics/ocs2

---

# Demonstration

![](.image/simulation_video_gif.gif)

---

# License

This project inherits licenses from the upstream repositories used in this work.
