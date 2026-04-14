#!/usr/bin/env bash
set -e

echo "====================================="
echo "OCS2 Quadruped Docker Build Script"
echo "====================================="

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$WS_DIR/src/Quadruped-Control-OCS2-ROS2"
MUJOCO_LIB="$SRC_DIR/mujoco/mujoco-3.2.2/lib"
QPOASES_DIR="$SRC_DIR/qpOASES-master"

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "ERROR: Required command '$1' is not installed or not on PATH."
        exit 1
    fi
}

echo "[0/6] Checking host prerequisites..."
require_command git
require_command docker
if ! docker compose version >/dev/null 2>&1; then
    echo "ERROR: 'docker compose' is not available."
    echo "Install Docker Compose plugin before running build.sh."
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    echo "ERROR: Docker daemon is not reachable."
    echo "Start Docker and make sure your user can run docker commands."
    exit 1
fi

echo "[1/6] Enable X11"
xhost +local:docker > /dev/null 2>&1 || true


echo "[2/6] Checking external dependencies..."

cd "$SRC_DIR"

clone_if_missing () {
    if [ -d "$1/.git" ]; then
        echo "$1 already exists"
    elif [ -d "$1" ]; then
        echo "ERROR: Directory '$1' already exists but is not a git checkout."
        echo "Remove it or convert it into a valid clone before running build.sh."
        exit 1
    else
        echo "Cloning $1..."
        git clone "$2"
    fi
}

clone_recursive_if_missing () {
    if [ -d "$1/.git" ]; then
        echo "$1 already exists"
    elif [ -d "$1" ]; then
        echo "ERROR: Directory '$1' already exists but is not a git checkout."
        echo "Remove it or convert it into a valid clone before running build.sh."
        exit 1
    else
        echo "Cloning $1 with submodules..."
        git clone --recurse-submodules "$2"
    fi
}

require_vendored_dir () {
    if [ ! -d "$1" ]; then
        echo "ERROR: Required vendored directory '$1' is missing."
        echo "This workspace now vendors '$1' directly into the main repository."
        echo "Re-clone the main repository or restore that directory before building."
        exit 1
    fi
    echo "$1 is vendored in this workspace"
}

require_vendored_dir "ocs2_ros2"
clone_if_missing "ocs2_robotic_assets" https://github.com/zhengxiang94/ocs2_robotic_assets.git
clone_if_missing "plane_segmentation_ros2" https://github.com/zhengxiang94/plane_segmentation_ros2.git
clone_recursive_if_missing "pinocchio" https://github.com/zhengxiang94/pinocchio.git
clone_recursive_if_missing "hpp-fcl" https://github.com/zhengxiang94/hpp-fcl.git

cd "$WS_DIR"


echo "[3/6] Checking MuJoCo library symlink..."

if [ -d "$MUJOCO_LIB" ]; then
    if [ -f "$MUJOCO_LIB/libmujoco.so.3.2.2" ] && [ ! -f "$MUJOCO_LIB/libmujoco.so" ]; then
        echo "Creating MuJoCo symlink..."
        ln -s libmujoco.so.3.2.2 "$MUJOCO_LIB/libmujoco.so"
    else
        echo "MuJoCo symlink already exists"
    fi
else
    echo "WARNING: MuJoCo directory not found:"
    echo "$MUJOCO_LIB"
fi


echo "[4/6] Building qpOASES..."

if [ -d "$QPOASES_DIR" ]; then
    cd "$QPOASES_DIR"

    mkdir -p build
    cd build

    cmake ..
    make -j$(nproc)

    cd "$WS_DIR"
else
    echo "ERROR: qpOASES-master not found!"
    exit 1
fi


echo "[5/6] Building Docker image"
docker compose build


echo "[6/6] Building workspace in container..."

docker compose run --rm quad_ocs2 bash -lc "
set -e

source /opt/ros/humble/setup.bash
cd /workspaces/quad_ocs2_ws

echo 'Cleaning previous build'
rm -rf build install log || true

echo 'Building OCS2 core packages'
colcon build \
  --packages-up-to ocs2_legged_robot_ros ocs2_self_collision_visualization \
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
echo "Typical commands:"
echo "./run.sh quad_mini_real sim estimated debug rviz gui"
echo "./run.sh quad_mini_real real estimated debug norviz nogui"
echo "./run_real_viewer.sh quad_mini_real"
