#!/bin/bash
set -e

if [ -n "${XDG_RUNTIME_DIR}" ]; then
  mkdir -p "${XDG_RUNTIME_DIR}"
  chmod 700 "${XDG_RUNTIME_DIR}"
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

if [ -f "${WS_DIR}/mujoco_env.sh" ]; then
  source "${WS_DIR}/mujoco_env.sh"
fi

exec "$@"
