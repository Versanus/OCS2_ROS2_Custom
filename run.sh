#!/bin/bash

# Stop on error
set -e

WS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

echo "Loading quad_ocs2_ws environment..."

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/local_setup.sh"
source "${WS_DIR}/mujoco_env.sh"

echo "Environment loaded ✅"
echo "Launching Quadruped Simulation..."

ros2 launch launch_simulation legged_robot_sqp.launch.py robot_type:=b2
