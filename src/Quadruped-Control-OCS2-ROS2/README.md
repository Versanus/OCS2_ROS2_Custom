# Quadruped-Control-OCS2-ROS2
## Introduction
This project is based on the open-source library of ocs2_ros2, setting up a simulation environment based on MuJoCo. It implements user command input, NMPC motion control, and quadruped robot physics simulation within the ROS 2 environment. 
![image](.image/structure.jpg)
![image](.image/mujoco_simulation_image.png)

## Installation
### Prerequisites
* Ubuntu 22.04
* ros2 humble
* C++ compiler with C++17 support
* Eigen (v3.4)
* Boost C++ (v1.74)

### Source code
The legged control project is developed based on the open-source library of [OCS2](https://github.com/leggedrobotics/ocs2.git) and [qiayuanl/legged_control](https://github.com/qiayuanl/legged_control.git)
```
# Clone legged_control
git clone https://github.com/HexiangZhou/Quadruped-Control-OCS2-ROS2.git
```

### OCS2_ROS2
The ocs2_ros2 library is based on [zhengxiang94/ocs2_ros2](https://github.com/zhengxiang94/ocs2_ros2.git)
```
# Clone ocs2_ros2 in ros2_ws/src/Quadruped-Control-OCS2-ROS2
cd ~/ros2_ws/src/Quadruped-Control-OCS2-ROS2
# Clone ocs2_ros2
git clone https://github.com/zhengxiang94/ocs2_ros2.git
# Clone pinocchio
git clone --recurse-submodules https://github.com/zhengxiang94/pinocchio.git
# Clone hpp-fcl
git clone --recurse-submodules https://github.com/zhengxiang94/hpp-fcl.git
# Clone ocs2_robotic_assets
git clone https://github.com/zhengxiang94/ocs2_robotic_assets.git
# Clone plane_segmentation_ros2
git clone https://github.com/zhengxiang94/plane_segmentation_ros2.git
```

### Others
```
# Install dependencies
sudo apt-get install ros-humble-grid-map-cv ros-humble-grid-map-msgs ros-humble-grid-map-ros ros-humble-grid-map-sdf ros-humble-octomap libmpfr-dev libpcap-dev libglpk-dev libglfw3-dev
```

## Docker
The repository now includes a Docker-based ROS 2 Humble environment that mounts this workspace and resolves the vendored MuJoCo and qpOASES paths automatically.

### Prerequisites
* Docker Engine
* Docker Compose plugin (`docker compose`)
* X11 on the host if you want to launch the MuJoCo GUI

### Build the image
```
xhost +SI:localuser:$(id -un)
env HOST_UID=$(id -u) HOST_GID=$(id -g) USER_NAME=$(id -un) docker compose build
```

### Open a shell in the container
```
docker compose run --rm quad_ocs2 bash
```

### Build the workspace in the container
```
colcon build --packages-up-to motion_control mujoco_simulator user_command launch_simulation --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Run the simulator in the container
```
docker compose run --rm quad_ocs2 ./run.sh
```

### Notes
* The container mounts the current workspace at `/workspaces/quad_ocs2_ws`.
* `mujoco_env.sh` is sourced automatically by the container entrypoint.
* The compose file defaults to software OpenGL with `LIBGL_ALWAYS_SOFTWARE=1`. If you have GPU passthrough configured, override it with `LIBGL_ALWAYS_SOFTWARE=0`.
* When you are finished with GUI sessions, you can revoke the X11 permission with:
```
xhost -SI:localuser:$(id -un)
```

## Build
### Build ocs2
```
# If downloading the dependencies via HTTPS fails, use SSH instead
git config --global url."git@github.com:".insteadOf https://github.com/
colcon build --packages-up-to ocs2_legged_robot_ros ocs2_self_collision_visualization --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
Ensure you can test ANYmal using command below:
```
source install/setup.bash
ros2 launch ocs2_legged_robot_ros legged_robot_sqp.launch.py
```
![](.image/ocs2_gif.gif)
### Build mujoco_simulator
MuJoCo is vendored in this repository, so no manual CMake path edits are required. Source the helper script first:
```
source mujoco_env.sh
```
Then build the package:
```
colcon build --packages-up-to mujoco_simulator
```
### Build motion_control
qpOASES is also vendored in this repository, so the package can be built directly:
```
colcon build --packages-up-to motion_control
```
### Build user_command
```
colcon build --packages-up-to user_command
```
### Build ros2 launch
```
colcon build --packages-up-to launch_simulation
```

## Quick Start
```
source install/local_setup.sh
# robot type: [a1,b1] (unitree a1, unitree b1)
ros2 launch launch_simulation legged_robot_sqp.launch.py robot_type:=b1
```
### Command
You can use the user_command shell to control the gait and movement of the legged robot.

**Examples:**
* Use "gait:trot" to switch the gait to trot.
* Use "goal:1 1 0 0" to set the movement goal to "(x, y, z, yaw)=(1, 1, 0, 0)".

**Demonstration:**
![](.image/simulation_video_gif.gif)
