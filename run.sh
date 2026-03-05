#!/bin/bash

# Stop on error
set -e

echo "Loading quad_ocs2_ws environment..."

source /opt/ros/humble/setup.bash
source ~/quad_ocs2_ws/install/local_setup.sh
source ~/quad_ocs2_ws/mujoco_env.sh

echo "Environment loaded ✅"
echo "Launching Quadruped Simulation..."

ros2 launch launch_simulation legged_robot_sqp.launch.py robot_type:=quad_mini
