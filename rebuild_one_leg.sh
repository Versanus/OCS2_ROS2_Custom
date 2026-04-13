#!/usr/bin/env bash

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

filter_existing_prefixes() {
    local var_name="$1"
    local current_value="${!var_name:-}"
    local filtered_value=""
    local entry

    IFS=':' read -r -a prefix_entries <<< "${current_value}"
    for entry in "${prefix_entries[@]}"; do
        if [ -n "${entry}" ] && [ -d "${entry}" ]; then
            filtered_value="${filtered_value:+${filtered_value}:}${entry}"
        fi
    done

    export "${var_name}=${filtered_value}"
}

if [ -f "/.dockerenv" ]; then
    echo "Running one-leg controller rebuild inside Docker"

    cd "${WS_DIR}"

    echo "Cleaning one_leg_pinocchio_control..."
    #rm -rf build/one_leg_pinocchio_control install/one_leg_pinocchio_control log/latest_build_one_leg

    echo "Loading environment..."
    source /opt/ros/humble/setup.bash

    if [ -f "${WS_DIR}/mujoco_env.sh" ]; then
        source "${WS_DIR}/mujoco_env.sh"
    fi

    if [ -f "${WS_DIR}/install/setup.bash" ]; then
        source "${WS_DIR}/install/setup.bash" || true
    fi

    filter_existing_prefixes AMENT_PREFIX_PATH
    filter_existing_prefixes CMAKE_PREFIX_PATH
    filter_existing_prefixes COLCON_PREFIX_PATH

    echo "Rebuilding one-leg controller only..."
    colcon build \
        --packages-select one_leg_pinocchio_control \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

    source install/local_setup.bash

    echo "One-leg controller rebuild finished"
    echo "Try:"
    echo "  ./run_one_leg.sh"
else
    echo "Running on host system"
    echo "Starting Docker container for one-leg rebuild..."

    docker compose run --rm quad_ocs2 ./rebuild_one_leg.sh
fi
