#!/usr/bin/env bash

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_TYPE="${1:-quad_mini_real}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ROS_DOMAIN_ID

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

    xhost +local:docker >/dev/null 2>&1 || true

    cleanup_done=false
    cleanup_container() {
        if [ "${cleanup_done}" = true ]; then
            return
        fi
        cleanup_done=true
        docker compose down --timeout 1 >/dev/null 2>&1 || true
    }

    handle_interrupt() {
        cleanup_container
        exit 130
    }

    trap cleanup_container EXIT
    trap handle_interrupt INT TERM

    docker compose up -d

    echo "Attaching to container..."
    docker exec -it -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        "$(docker compose ps -q quad_ocs2)" \
        ./run_real_viewer.sh "${ROBOT_TYPE}"
fi
