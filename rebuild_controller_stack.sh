#!/usr/bin/env bash

set -euo pipefail

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

safe_source() {
    local script_path="$1"
    set +u
    # shellcheck disable=SC1090
    source "${script_path}"
    set -u
}

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
    echo "Running controller stack rebuild inside Docker"

    cd "${WS_DIR}"

    echo "Loading environment..."
    safe_source /opt/ros/humble/setup.bash
    safe_source "${WS_DIR}/mujoco_env.sh"

    if [ -f "${WS_DIR}/install/setup.bash" ]; then
        safe_source "${WS_DIR}/install/setup.bash"
    fi

    filter_existing_prefixes AMENT_PREFIX_PATH
    filter_existing_prefixes CMAKE_PREFIX_PATH
    filter_existing_prefixes COLCON_PREFIX_PATH

    echo "Rebuilding changed controller packages..."
    colcon build \
        --packages-select stm2ros legged_msgs motion_control mujoco_simulator real_robot_bridge hardware_interface user_command launch_simulation

    safe_source install/local_setup.bash
    echo "Controller stack rebuild finished"
else
    echo "Running on host system"
    echo "Starting Docker container for controller stack rebuild..."
    docker compose run --rm quad_ocs2 ./rebuild_controller_stack.sh
fi
