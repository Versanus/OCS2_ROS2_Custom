#!/usr/bin/env bash

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ROS_DOMAIN_ID

if [ -f "/.dockerenv" ]; then
    echo "Running one-leg RViz viewer inside Docker"

    cd "${WS_DIR}"

    if [ ! -d "${WS_DIR}/install/one_leg_pinocchio_control" ]; then
        echo "one_leg_pinocchio_control is not built yet. Run ./rebuild_one_leg.sh first."
        exit 1
    fi

    echo "Loading workspace environment..."
    source /opt/ros/humble/setup.bash
    source "${WS_DIR}/install/local_setup.bash"

    echo "Launching one-leg RViz viewer on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}..."
    echo "Extra launch args: $*"

    ros2 launch one_leg_pinocchio_control one_leg_rviz.launch.py \
        config_file:="${WS_DIR}/src/one_leg_pinocchio_control/config/one_leg_inverse_dynamics.yaml" \
        urdf_path:="${WS_DIR}/src/one_leg_pinocchio_control/urdf/bacak_test_description.urdf" \
        rviz_config:="${WS_DIR}/src/one_leg_pinocchio_control/rviz/one_leg.rviz" \
        "$@"
else
    echo "Running on host system"
    echo "Starting Docker container for one-leg RViz viewer..."

    xhost +local:docker >/dev/null 2>&1 || true

    docker compose run --rm \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        quad_ocs2 \
        ./run_one_leg_rviz.sh "$@"
fi
