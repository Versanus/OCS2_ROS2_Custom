#!/bin/bash

set -e

WS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

cd "${WS_DIR}"

rm -rf build/mujoco_simulator
rm -rf install/mujoco_simulator

rm -rf build/user_command
rm -rf install/user_command

source /opt/ros/humble/setup.bash
source install/setup.bash
source mujoco_env.sh

colcon build --packages-select mujoco_simulator user_command --symlink-install --cmake-clean-cache

source install/local_setup.sh
