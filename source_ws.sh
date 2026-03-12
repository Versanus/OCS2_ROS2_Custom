#!/usr/bin/env bash

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "Loading quad_ocs2_ws environment..."

# Load ROS2
source /opt/ros/humble/setup.bash

# Ensure workspace is built
if [ ! -d "${WS_DIR}/install" ]; then
    echo "Workspace not built. Run ./build.sh first."
    return 1 2>/dev/null || exit 1
fi

# Load workspace
source "${WS_DIR}/install/local_setup.bash"

# Load MuJoCo paths
source "${WS_DIR}/mujoco_env.sh"

echo "quad_ocs2_ws environment loaded ✅"