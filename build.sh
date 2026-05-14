#!/usr/bin/env bash
set -e

echo "====================================="
echo "OCS2 Quadruped Docker Build Script"
echo "====================================="

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$WS_DIR/src/Quadruped-Control-OCS2-ROS2"
MUJOCO_LIB="$SRC_DIR/mujoco/mujoco-3.2.2/lib"
QPOASES_DIR="$SRC_DIR/qpOASES-master"
HOST_UID="${HOST_UID:-$(id -u)}"
HOST_GID="${HOST_GID:-$(id -g)}"
USER_NAME="${USER_NAME:-$(id -un)}"

if [ -z "${INPUT_GID:-}" ]; then
    if getent group input >/dev/null 2>&1; then
        INPUT_GID="$(getent group input | cut -d: -f3)"
    elif [ -e /dev/input ]; then
        INPUT_GID="$(stat -c '%g' /dev/input)"
    else
        INPUT_GID="${HOST_GID}"
    fi
fi

export HOST_UID HOST_GID USER_NAME INPUT_GID

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "ERROR: Required command '$1' is not installed or not on PATH."
        exit 1
    fi
}

echo "[0/6] Checking host prerequisites..."
require_command git
require_command docker

echo "[1/6] Enable X11"
xhost +local:docker > /dev/null 2>&1 || true


echo "[2/6] Checking external dependencies..."

if [ ! -d "$SRC_DIR/ocs2_ros2" ]; then
    echo "ERROR: Required vendored directory '$SRC_DIR/ocs2_ros2' is missing."
    echo "Re-clone the main repository or restore that directory before building."
    exit 1
fi

if [ ! -f "$WS_DIR/.gitmodules" ]; then
    echo "ERROR: .gitmodules is missing. Cannot initialize pinned external repositories."
    exit 1
fi

git -C "$WS_DIR" submodule update --init --recursive \
    src/Quadruped-Control-OCS2-ROS2/ocs2_robotic_assets \
    src/Quadruped-Control-OCS2-ROS2/plane_segmentation_ros2 \
    src/Quadruped-Control-OCS2-ROS2/pinocchio \
    src/Quadruped-Control-OCS2-ROS2/hpp-fcl


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

echo 'Building simulator, bridge, and visualization packages'
colcon build --packages-select \
  legged_msgs \
  stm2ros \
  inekf \
  motion_control \
  hardware_inter \
  gazebo_effort_controller \
  mujoco_simulator \
  real_robot_bridge \
  user_command \
  launch_simulation
"

echo ""
echo "====================================="
echo "Build finished successfully"
echo "====================================="
echo ""
echo "Typical commands:"
echo "./run.sh quad_mini_tuned sim mujoco debug rviz gui"
echo "./run.sh quad_mini_real real estimated debug norviz nogui"
echo "./run_viewer hardware quad_mini_real"
