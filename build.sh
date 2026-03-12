#!/usr/bin/env bash

set -e

echo "====================================="
echo "OCS2 Quadruped Docker Build Script"
echo "====================================="

# allow docker GUI access
echo "[1/4] Enabling X11 access for Docker..."
xhost +local:docker > /dev/null

# build docker image
echo "[2/4] Building Docker image..."
docker compose build

# build ROS workspace inside container
echo "[3/4] Building ROS2 workspace..."

docker compose run --rm quad_ocs2 bash -c "
colcon build \
  --packages-up-to motion_control mujoco_simulator user_command launch_simulation \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
"

echo "[4/4] Build finished successfully!"
echo ""
echo "To start simulation run:"
echo "./run.sh"
