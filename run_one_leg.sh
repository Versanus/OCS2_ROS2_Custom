#!/usr/bin/env bash

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ROS_DOMAIN_ID

variant="default"
variant_args=()
if [ "${1:-}" = "go2" ]; then
    variant="go2"
    variant_args=("go2")
    shift
fi

if [ -f "/.dockerenv" ]; then
    echo "Running one-leg controller inside Docker"

    cd "${WS_DIR}"

    GUI_EXEC="${WS_DIR}/install/one_leg_pinocchio_control/lib/one_leg_pinocchio_control/one_leg_control_gui.py"
    CONTROLLER_EXEC="${WS_DIR}/install/one_leg_pinocchio_control/lib/one_leg_pinocchio_control/one_leg_inverse_dynamics_node"
    STATE_MAPPER_EXEC="${WS_DIR}/install/one_leg_pinocchio_control/lib/one_leg_pinocchio_control/one_leg_state_to_model_joint_state.py"

    if [ ! -x "${CONTROLLER_EXEC}" ] || [ ! -x "${GUI_EXEC}" ] || [ ! -x "${STATE_MAPPER_EXEC}" ]; then
        echo "one_leg_pinocchio_control install is missing controller, GUI, or RViz state mapper executable."
        echo "Run ./rebuild_one_leg.sh first, then run ./run_one_leg.sh again."
        exit 1
    fi

    echo "Loading workspace environment..."
    source /opt/ros/humble/setup.bash
    source "${WS_DIR}/install/local_setup.bash"

    CONFIG_FILE="${WS_DIR}/src/one_leg_pinocchio_control/config/one_leg_inverse_dynamics.yaml"
    URDF_FILE="${WS_DIR}/src/one_leg_pinocchio_control/urdf/bacak_test_description.urdf"
    if [ "${variant}" = "go2" ]; then
        CONFIG_FILE="${WS_DIR}/src/one_leg_pinocchio_control/config/one_leg_inverse_dynamics_go2.yaml"
        URDF_FILE="${WS_DIR}/src/one_leg_pinocchio_control/urdf/go2_one_leg.urdf"
    fi

    echo "Launching one-leg controller panel (${variant}) on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
    echo "Extra launch args: $*"

    ros2 launch one_leg_pinocchio_control one_leg_control_panel.launch.py \
        config_file:="${CONFIG_FILE}" \
        urdf_path:="${URDF_FILE}" \
        rviz_config:="${WS_DIR}/src/one_leg_pinocchio_control/rviz/one_leg.rviz" \
        "$@"
else
    echo "Running on host system"
    echo "Starting Docker container for one-leg controller..."

    xhost +local:docker >/dev/null 2>&1 || true

    docker compose run --rm \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        quad_ocs2 \
        ./run_one_leg.sh "${variant_args[@]}" "$@"
fi
