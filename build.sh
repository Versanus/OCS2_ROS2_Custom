#!/usr/bin/env bash
set -e

echo "====================================="
echo "OCS2 Quadruped Docker Build Script"
echo "====================================="

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$WS_DIR/src/Quadruped-Control-OCS2-ROS2"

echo "[1/4] Enable X11"
xhost +local:docker > /dev/null 2>&1 || true

echo "[2/4] Checking external dependencies..."

cd "$SRC_DIR"

clone_if_missing () {
if [ ! -d "$1/.git" ]; then
echo "Cloning $1..."
git clone "$2"
else
echo "$1 already exists"
fi
}

clone_if_missing "ocs2_ros2" https://github.com/zhengxiang94/ocs2_ros2.git
clone_if_missing "ocs2_robotic_assets" https://github.com/zhengxiang94/ocs2_robotic_assets.git
clone_if_missing "plane_segmentation_ros2" https://github.com/zhengxiang94/plane_segmentation_ros2.git

if [ ! -d pinocchio/.git ]; then
echo "Cloning pinocchio..."
git clone --recurse-submodules https://github.com/zhengxiang94/pinocchio.git
fi

if [ ! -d hpp-fcl/.git ]; then
echo "Cloning hpp-fcl..."
git clone --recurse-submodules https://github.com/zhengxiang94/hpp-fcl.git
fi

cd "$WS_DIR"

echo "[3/4] Building Docker image"
docker compose build

MUJOCO_LIB="src/Quadruped-Control-OCS2-ROS2/mujoco/mujoco-3.2.2/lib"

if [ -f "$MUJOCO_LIB/libmujoco.so.3.2.2" ] && [ ! -f "$MUJOCO_LIB/libmujoco.so" ]; then
    echo "Creating MuJoCo library symlink..."
    ln -s libmujoco.so.3.2.2 "$MUJOCO_LIB/libmujoco.so"
fi

# build ROS workspace inside container
echo "[3/4] Building ROS2 workspace..."

docker compose run --rm 
-u $(id -u):$(id -g) 
-v "$WS_DIR:/workspaces/quad_ocs2_ws" 
quad_ocs2 bash -c "

set -e
source /opt/ros/humble/setup.bash
cd /workspaces/quad_ocs2_ws

echo 'Cleaning previous build'
rm -rf build install log || true

echo 'Building OCS2 core packages'
colcon build 
--packages-up-to ocs2_legged_robot_ros ocs2_self_collision_visualization 
--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo 'Sourcing workspace'
source install/setup.bash

echo 'Building simulator packages'
colcon build --packages-up-to mujoco_simulator
colcon build --packages-up-to motion_control
colcon build --packages-up-to user_command
colcon build --packages-up-to launch_simulation
"

echo ""
echo "====================================="
echo "Build finished successfully"
echo "====================================="
echo ""
echo "Run the simulator with:"
echo "./run.sh"
