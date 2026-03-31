#!/usr/bin/env bash

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Detect if running inside Docker
if [ -f "/.dockerenv" ]; then
    echo "Running quick rebuild inside Docker"

    cd "${WS_DIR}"

    echo "Cleaning selected packages..."

    rm -rf build/user_command install/user_command
    rm -rf build/launch_simulation install/launch_simulation

    echo "Loading environment..."

    source /opt/ros/humble/setup.bash
    source "${WS_DIR}/mujoco_env.sh"

    if [ -f "${WS_DIR}/install/setup.bash" ]; then
        source "${WS_DIR}/install/setup.bash"
    fi

    echo "Rebuilding packages..."

    colcon build \
        --packages-select user_command launch_simulation \
        --symlink-install \
        --cmake-clean-cache

    source install/local_setup.bash

    echo "Quick rebuild finished ✅"

else
    echo "Running on host system"
    echo "Starting Docker container for quick rebuild..."

    docker compose run --rm quad_ocs2 ./rebuild_user_command.sh
fi
