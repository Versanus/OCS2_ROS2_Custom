# Quadruped-Control-OCS2-ROS2

## Overview

This project implements a **quadruped robot simulation and control framework** using:

- **ROS 2 (Humble)**
- **OCS2 (Optimal Control for Switched Systems)**
- **MuJoCo physics simulator**

The system integrates:

- user command interface  
- nonlinear model predictive control (**NMPC**)  
- quadruped robot physics simulation  
- ROS 2 communication and launch infrastructure  

The controller is based on **centroidal NMPC using OCS2** and runs in real time inside a MuJoCo simulation environment.

This repository builds upon several open-source projects:

- https://github.com/leggedrobotics/ocs2  
- https://github.com/qiayuanl/legged_control  
- https://github.com/zhengxiang94/ocs2_ros2  

---

# Simulation Architecture

The overall system architecture is illustrated below.

![structure](.image/structure.jpg)

Example MuJoCo simulation environment:

![mujoco](.image/mujoco_simulation_image.png)

---

# Quick Start (Docker)

The easiest way to run the project is using **Docker**, which automatically manages all dependencies.

## Requirements

- Ubuntu 22.04  
- Docker Engine  
- Docker Compose plugin (`docker compose`)  
- X11 display server (for MuJoCo viewer)

Install Docker if necessary:

```bash
sudo apt install docker.io docker-compose-plugin
```

---

## Clone the Repository

```bash
git clone https://github.com/Versanus/OCS2_quad_mini.git
cd OCS2_quad_mini
```

---

## Build the Environment

Run:

```bash
./build.sh
```

This script will:

- build the Docker image  
- build the ROS2 workspace  
- configure MuJoCo  
- compile qpOASES  
- fetch required external repositories  

The first build may take several minutes.

---

## Run the Simulation

Start the quadruped simulation:

```bash
./run.sh
```

This launches:

- MuJoCo simulator  
- NMPC controller  
- user command interface  

If X11 is configured correctly, the **MuJoCo viewer window should appear**.

---

# Manual Docker Workflow (Optional)

You can also run the project manually using Docker commands.

---

## Enable X11 Access

```bash
xhost +local:docker
```

---

## Build Docker Image

```bash
docker compose build
```

---

## Enter the Container

```bash
docker compose run --rm quad_ocs2 bash
```

---

## Build the ROS2 Workspace

Inside the container:

```bash
colcon build \
  --packages-up-to motion_control mujoco_simulator user_command launch_simulation \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

---

## Launch the Simulation

```bash
ros2 launch launch_simulation simulation.launch.py
```

---

# Simulation Controls

The **user_command** interface allows sending commands to the robot during runtime.

Example commands:

```
gait:trot
goal:1 1 0 0
```

Command format:

```
goal: x y z yaw
```

Example:

```
goal:1 1 0 0
```

This moves the robot toward the position:

```
x = 1
y = 1
z = 0
yaw = 0
```

---

# Project Structure

```
OCS2_quad_mini
│
├── docker/                  Docker environment configuration
├── src/                     ROS2 packages
├── tools/                   helper utilities
│
├── build.sh                 builds Docker image and workspace
├── run.sh                   launches the simulation
├── rebuild_quick.sh         fast rebuild for development
├── docker-compose.yml
│
└── README.md
```

---

# Dependencies

The Docker environment automatically installs all required dependencies, including:

- ROS2 Humble  
- MuJoCo  
- OCS2  
- qpOASES  
- grid_map  
- OctoMap  
- Pinocchio  
- HPP-FCL  

---

# Development

For faster rebuilds during development:

```bash
./rebuild_quick.sh
```

This script rebuilds only the required packages instead of the entire workspace.

---

# Troubleshooting

## MuJoCo window does not appear

Ensure X11 permissions are enabled:

```bash
xhost +local:docker
```

---

## Check running ROS nodes

Inside the container:

```bash
ros2 node list
```

Expected nodes:

```
/mujoco_simulator
/user_command_node
/legged_robot_sqp_mpc
```

---

# References

This project builds upon the following open-source work:

- https://github.com/HexiangZhou/Quadruped-Control-OCS2-ROS2  
- https://github.com/zhengxiang94/ocs2_ros2  
- https://github.com/leggedrobotics/ocs2  

---

# Demonstration

![](.image/simulation_video_gif.gif)

---

# License

This repository inherits licenses from the upstream projects used in this work.
