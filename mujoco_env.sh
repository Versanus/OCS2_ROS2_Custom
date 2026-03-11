#!/bin/bash

WS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
MUJOCO_ROOT="${WS_DIR}/src/Quadruped-Control-OCS2-ROS2/mujoco/mujoco-3.2.2"

export LD_LIBRARY_PATH="${MUJOCO_ROOT}/lib:${LD_LIBRARY_PATH}"
export PATH="${MUJOCO_ROOT}/bin:${PATH}"
