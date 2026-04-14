#!/usr/bin/env bash

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_TYPE="${1:-quad_mini_real}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ROS_DOMAIN_ID

require_host_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo "ERROR: Required command '$1' is not installed or not on PATH."
        exit 1
    fi
}

if [ -f "/.dockerenv" ]; then
    echo "Running real-hardware viewer inside Docker"

    if [ ! -d "${WS_DIR}/install" ]; then
        echo "Workspace not built. Run ./build.sh first."
        exit 1
    fi

    source /opt/ros/humble/setup.bash
    source "${WS_DIR}/install/local_setup.sh"

    ./tools/run_real_viewer_tmux.sh "${ROBOT_TYPE}"
else
    echo "Running real-hardware viewer on host"
    echo "Starting Docker container..."

    require_host_command docker
    if ! docker compose version >/dev/null 2>&1; then
        echo "ERROR: 'docker compose' is not available."
        exit 1
    fi
    if ! docker info >/dev/null 2>&1; then
        echo "ERROR: Docker daemon is not reachable."
        exit 1
    fi

    xhost +local:docker >/dev/null 2>&1 || true
    docker compose up -d

    echo "Attaching to container..."
    docker exec -it -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        "$(docker compose ps -q quad_ocs2)" \
        ./run_real_viewer.sh "${ROBOT_TYPE}"
fi
