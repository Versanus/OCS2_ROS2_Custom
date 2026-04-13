#!/usr/bin/env bash

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

load_quad_ocs2_ws() {
    echo "Loading quad_ocs2_ws environment..."

    ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
    export ROS_DOMAIN_ID

    # Load ROS2
    source /opt/ros/humble/setup.bash

    # Ensure workspace is built
    if [ ! -d "${WS_DIR}/install" ]; then
        echo "Workspace not built. Run ./build.sh first."
        return 1
    fi

    # Load workspace. The bash variant resolves the current prefix correctly
    # even if the workspace was built in Docker under a different path.
    source_local_setup_bash

    # Load MuJoCo paths
    source "${WS_DIR}/mujoco_env.sh"

    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "quad_ocs2_ws environment loaded ✅"
}

source_local_setup_bash() {
    local setup_stderr
    setup_stderr="$(mktemp)"

    if ! source "${WS_DIR}/install/local_setup.bash" 2>"${setup_stderr}"; then
        cat "${setup_stderr}" >&2
        rm -f "${setup_stderr}"
        return 1
    fi

    grep -vE '^not found: ".*/install/[^/]+/share/[^/]+/local_setup\.bash"$' "${setup_stderr}" >&2 || true
    rm -f "${setup_stderr}"
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "This script must be sourced so the environment stays in your shell."
    echo "Use: source ./source_ws.sh"
    exit 1
fi

load_quad_ocs2_ws
