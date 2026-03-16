# Quadruped-Control-OCS2-ROS2

A **complete quadruped robot simulation and control framework** built with:

* **ROS 2 (Humble Hawksbill)**
* **OCS2 (Optimal Control for Switched Systems)**
* **MuJoCo physics simulator**
* **Nonlinear Model Predictive Control (NMPC)**

This project provides a **fully reproducible quadruped locomotion environment** where a robot is simulated in **MuJoCo** and controlled in real time using **OCS2-based NMPC**.

The entire system runs inside **Docker**, meaning you **do not need to manually install ROS2, MuJoCo, or OCS2**.

Everything required to run the simulator is automatically configured.

---

# Table of Contents

1. Overview
2. System Architecture
3. Features
4. System Requirements
5. Installation
6. Building the Project
7. Running the Simulation
8. Selecting Different Robots
9. Controlling the Robot
10. Manual Launch Without `run.sh`
11. ROS System Overview
12. Project Structure
13. Script Explanations
14. Development Workflow
15. Debugging Tools
16. Troubleshooting
17. References
18. License

---

# Overview

This repository simulates and controls quadruped robots using **centroidal Nonlinear Model Predictive Control (NMPC)**.

The system integrates four major components:

1. **MuJoCo Simulator** — simulates the robot physics
2. **OCS2 Controller** — computes optimal control actions
3. **ROS2 Infrastructure** — communication between modules
4. **User Command Interface** — runtime robot control

The NMPC controller continuously solves an **optimal control problem in real time**, allowing the robot to walk, trot, and move based on commands.

The framework is designed for:

* robotics research
* legged locomotion experiments
* optimal control research
* robotics education
* NMPC algorithm testing

---

# System Architecture

The system contains three main subsystems.

---

## MuJoCo Simulator

The MuJoCo simulator is responsible for:

* simulating rigid body dynamics
* modeling ground contacts
* computing physics interactions
* visualizing the robot
* publishing robot state information to ROS2

---

## NMPC Controller (OCS2)

The controller:

* uses **centroidal dynamics**
* solves a **nonlinear optimal control problem**
* runs continuously
* outputs optimal joint torques

The controller interacts with the simulator using ROS topics.

---

## User Command Interface

This module allows users to interact with the robot during simulation.

Users can:

* change gait
* send movement commands
* test controller behavior

---

# Features

* Real-time **Nonlinear Model Predictive Control**
* **MuJoCo physics simulation**
* Modular **ROS2 architecture**
* Interactive command interface
* Multiple quadruped robot models
* Fully reproducible **Docker environment**
* **tmux-based multi-terminal interface**
* Easy robot switching
* Development-friendly rebuild scripts

---

# System Requirements

Recommended system specifications:

| Component | Requirement  |
| --------- | ------------ |
| OS        | Ubuntu 22.04 |
| RAM       | 8 GB minimum |
| Storage   | 15 GB free   |
| GPU       | Optional     |

---

# Installation

## Step 1 — Install Docker

Update your system:

```
sudo apt update
```

Install Docker:

```
sudo apt install docker.io docker-compose-plugin
```

Add your user to the docker group:

```
sudo usermod -aG docker $USER
```

Log out and log back in.

Verify installation:

```
docker --version
docker compose version
```

---

# Step 2 — Clone the Repository

Clone the project:

```
git clone https://github.com/Versanus/OCS2_quad_mini.git
```

Enter the directory:

```
cd OCS2_quad_mini
```

---

# Building the Project

Run the build script:

```
./build.sh
```

This script automatically performs the following steps:

1. Enables X11 display access
2. Clones required external repositories
3. Builds qpOASES optimization library
4. Builds the Docker image
5. Compiles the ROS2 workspace inside Docker

The build process may take **10–20 minutes**.

When finished, you will see:

```
Build finished successfully
```

---

# Running the Simulation

The easiest way to start the simulator is:

```
./run.sh
```

This script automatically:

1. Starts the Docker container
2. Loads ROS2 environment
3. Loads MuJoCo libraries
4. Launches tmux workspace
5. Starts simulation nodes

---

# tmux Simulation Interface

The system launches **four terminals using tmux**.

```
+----------------------+----------------------+
| Mujoco Simulator     | User Command Input   |
+----------------------+----------------------+
| NMPC Controller      | Debug Terminal       |
+----------------------+----------------------+
```

Terminal functions:

| Terminal         | Purpose                 |
| ---------------- | ----------------------- |
| Mujoco Simulator | runs physics simulation |
| User Command     | send commands to robot  |
| NMPC Controller  | runs optimal controller |
| Debug Terminal   | debugging ROS tools     |

---

# Selecting Different Robots

Supported robots:

```
a1
go1
b1
b2
```

Run a specific robot:

```
./run.sh go1
```

Default robot:

```
b1
```

---

# Controlling the Robot

Commands are entered in the **User Command terminal**.

---

## List Available Gaits

```
gait:list
```

Example output:

```
standing
walking
flying_trot
dynamic_walk
```

---

## Change Robot Gait

Example:

```
gait:flying_trot
```

---

## Send Motion Commands

Command format:

```
goal:x y z yaw
```

Example:

```
goal:1 0 0 0
```

Meaning:

| Parameter | Description     |
| --------- | --------------- |
| x         | forward motion  |
| y         | sideways motion |
| z         | body height     |
| yaw       | robot rotation  |

---

# Running Without `run.sh`

Advanced users may want to run everything manually.

First start the container:

```
docker compose run --rm quad_ocs2 bash
```

Inside the container load the environment:

```
source /opt/ros/humble/setup.bash
source install/setup.bash
source mujoco_env.sh
```

---

# Start MuJoCo Simulator

```
ros2 launch launch_simulation mujoco.launch.py robot_type:=b1
```

---

# Start User Command Node

```
REF=$(ros2 pkg prefix user_command)/share/user_command/config/b1/reference.info
GAIT=$(ros2 pkg prefix user_command)/share/user_command/config/b1/gait.info

ros2 run user_command user_command_node \
 --ros-args \
 -p referenceFile:=$REF \
 -p gaitCommandFile:=$GAIT
```

---

# Start MPC Controller

```
ros2 launch launch_simulation mpc.launch.py robot_type:=b1
```

---

# Checking ROS Nodes

Verify the system is running:

```
ros2 node list
```

Expected output:

```
/mujoco_simulator
/user_command_node
/legged_robot_sqp_mpc
```

---

# Project Structure

```
OCS2_quad_mini
│
├── docker
│
├── src
│   ├── motion_control
│   ├── mujoco_simulator
│   ├── user_command
│   └── launch_simulation
│
├── tools
│
├── build.sh
├── run.sh
├── rebuild_quick.sh
├── docker-compose.yml
│
└── README.md
```

---

# Script Explanations

## build.sh

Responsible for building the entire environment.

Steps:

* clones dependencies
* builds qpOASES
* builds Docker image
* compiles ROS workspace

Run once during installation.

```
./build.sh
```

---

## run.sh

Main simulation launcher.

Handles:

* Docker startup
* environment setup
* tmux launch

```
./run.sh
```

---

## rebuild_quick.sh

Used during development.

Rebuilds only selected packages:

```
mujoco_simulator
user_command
launch_simulation
```

Run:

```
./rebuild_quick.sh
```

---

## tools/run_tmux.sh

Creates tmux environment and launches nodes.

```
./tools/run_tmux.sh
```

---

# Development Workflow

Typical workflow:

Initial setup:

```
git clone <repo>
cd <repo>
./build.sh
```

Run simulation:

```
./run.sh
```

Rebuild after code changes:

```
./rebuild_quick.sh
```

---

# Debugging Tools

List nodes:

```
ros2 node list
```

List topics:

```
ros2 topic list
```

Inspect robot state:

```
ros2 topic echo /legged_robot_mpc_observation
```

Inspect MPC output:

```
ros2 topic echo /legged_robot_mpc_policy
```

---

# Resetting the Simulation

Stop tmux sessions:

```
tmux kill-server
```

Restart:

```
./run.sh
```

---

# Troubleshooting

## MuJoCo window does not appear

Enable X11:

```
xhost +local:docker
```

Restart simulation.

---

## ROS nodes missing

Check nodes:

```
ros2 node list
```

Expected:

```
/mujoco_simulator
/user_command_node
/legged_robot_sqp_mpc
```

---

# References

OCS2:

https://github.com/leggedrobotics/ocs2

Legged Control:

https://github.com/qiayuanl/legged_control

OCS2 ROS2:

https://github.com/zhengxiang94/ocs2_ros2

---

# License

This project inherits licenses from the upstream repositories used in this work.

Please refer to the original repositories for licensing information.

---

