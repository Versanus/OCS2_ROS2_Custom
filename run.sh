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

    ./tools/run_tmux.sh a1_custom

else
    echo "Running on host system"
    echo "Starting Docker container..."

    xhost +local:docker >/dev/null 2>&1 || true

    docker compose up -d

    echo "Attaching to container..."

    docker exec -it $(docker compose ps -q quad_ocs2) ./run.sh 
fi