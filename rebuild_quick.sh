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

# Detect if running inside Docker
if [ -f "/.dockerenv" ]; then
    echo "Running quick rebuild inside Docker"

    cd "${WS_DIR}"

    echo "Cleaning selected packages..."

    # rm -rf build/ocs2_oc install/ocs2_oc
    # rm -rf build/stm2ros install/stm2ros
    # rm -rf build/legged_msgs install/legged_msgs
    # rm -rf build/inekf install/inekf
    # rm -rf build/motion_control install/motion_control
    # rm -rf build/gazebo_effort_controller install/gazebo_effort_controller
    # rm -rf build/hardware_inter install/hardware_inter
    # rm -rf build/mujoco_simulator install/mujoco_simulator
    # rm -rf build/real_robot_bridge install/real_robot_bridge
    # rm -rf build/user_command install/user_command
    # rm -rf build/launch_simulation install/launch_simulation

    echo "Loading environment..."

    source /opt/ros/humble/setup.bash
    source "${WS_DIR}/mujoco_env.sh"

    if [ -f "${WS_DIR}/install/setup.bash" ]; then
        source "${WS_DIR}/install/setup.bash"
    fi

    filter_existing_prefixes AMENT_PREFIX_PATH
    filter_existing_prefixes CMAKE_PREFIX_PATH
    filter_existing_prefixes COLCON_PREFIX_PATH

    echo "Rebuilding packages..."

    colcon build \
        --packages-select ocs2_oc ocs2_mpc legged_msgs stm2ros inekf motion_control hardware_inter gazebo_effort_controller mujoco_simulator real_robot_bridge user_command launch_simulation
        #--cmake-clean-cache

    source install/local_setup.bash

    echo "Quick rebuild finished ✅"

else
    echo "Running on host system"
    echo "Starting Docker container for quick rebuild..."

    docker compose run --rm quad_ocs2 ./rebuild_quick.sh
fi
