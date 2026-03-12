#!/usr/bin/env bash

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Detect if running inside Docker
if [ -f "/.dockerenv" ]; then
    echo "Running inside Docker container"

    if [ ! -d "${WS_DIR}/install" ]; then
        echo "Workspace not built. Run ./build.sh first."
        exit 1
    fi

    echo "Loading quad_ocs2_ws environment..."

    source /opt/ros/humble/setup.bash
    source "${WS_DIR}/install/local_setup.sh"
    source "${WS_DIR}/mujoco_env.sh"

    echo "Environment loaded ✅"
    echo "Launching Quadruped Simulation..."

    ros2 launch launch_simulation legged_robot_sqp.launch.py robot_type:=b2

else
    echo "Running on host system"
    echo "Starting Docker container..."

    xhost +local:docker >/dev/null 2>&1 || true

    docker compose run --rm quad_ocs2 ./run.sh
fi