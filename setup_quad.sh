#!/bin/bash

WS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/local_setup.sh"
source "${WS_DIR}/mujoco_env.sh"

echo "quad_ocs2_ws environment loaded ✅"
