#!/usr/bin/env bash
#./run.sh quad_mini sim
#./run.sh quad_mini sim debug
#./run.sh quad_mini sim mujoco debug
#./run.sh quad_mini sim mujoco debug rviz gui
#./run.sh quad_mini sim mujoco nodebug norviz nogui
#./run.sh quad_mini_real real estimated debug rviz gui

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_TYPE="${1:-quad_mini}"
BACKEND="${2:-sim}"
CONTACT_SOURCE="${3:-}"
DEBUG_STATE_LOGGING="${4:-}"
RVIZ_AUTO="${5:-}"
GUI_AUTO="${6:-}"
RVIZ_SOURCE="${7:-auto}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ROS_DOMAIN_ID

if [ "${CONTACT_SOURCE}" = "debug" ] || [ "${CONTACT_SOURCE}" = "nodebug" ] || \
   [ "${CONTACT_SOURCE}" = "true" ] || [ "${CONTACT_SOURCE}" = "false" ]; then
    DEBUG_STATE_LOGGING="${CONTACT_SOURCE}"
    CONTACT_SOURCE=""
fi

if [ -z "${CONTACT_SOURCE}" ]; then
    if [ "${BACKEND}" = "real" ]; then
        CONTACT_SOURCE="estimated"
    else
        CONTACT_SOURCE="mujoco"
    fi
fi

case "${BACKEND}" in
    sim|real) ;;
    *)
        echo "Invalid backend: ${BACKEND}. Use 'sim' or 'real'."
        exit 1
        ;;
esac

case "${CONTACT_SOURCE}" in
    mujoco|estimated) ;;
    *)
        echo "Invalid contact source: ${CONTACT_SOURCE}. Use 'mujoco' or 'estimated'."
        exit 1
        ;;
    esac

case "${DEBUG_STATE_LOGGING}" in
    ""|nodebug|false)
        DEBUG_STATE_LOGGING="false"
        ;;
    debug|true)
        DEBUG_STATE_LOGGING="true"
        ;;
    *)
        echo "Invalid debug flag: ${DEBUG_STATE_LOGGING}. Use 'debug', 'nodebug', 'true', or 'false'."
        exit 1
        ;;
esac

case "${RVIZ_AUTO}" in
    ""|rviz|true)
        RVIZ_AUTO="true"
        ;;
    norviz|false)
        RVIZ_AUTO="false"
        ;;
    *)
        echo "Invalid RViz flag: ${RVIZ_AUTO}. Use 'rviz', 'norviz', 'true', or 'false'."
        exit 1
        ;;
esac

case "${GUI_AUTO}" in
    ""|gui|true)
        GUI_AUTO="true"
        ;;
    nogui|false)
        GUI_AUTO="false"
        ;;
    *)
        echo "Invalid GUI flag: ${GUI_AUTO}. Use 'gui', 'nogui', 'true', or 'false'."
        exit 1
        ;;
esac

case "${RVIZ_SOURCE}" in
    auto|sim|hardware)
        ;;
    *)
        echo "Invalid RViz source: ${RVIZ_SOURCE}. Use 'auto', 'sim', or 'hardware'."
        exit 1
        ;;
esac

# Detect if running inside Docker
if [ -f "/.dockerenv" ]; then
    echo "Running inside Docker container"

    if [ ! -d "${WS_DIR}/install" ]; then
        echo "Workspace not built. Run ./build.sh first."
        exit 1
    fi

    echo "Loading quad_ocs2_ws environment..."

    source /opt/ros/humble/setup.bash
    source "${WS_DIR}/install/local_setup.sh"
    if [ "${BACKEND}" != "real" ]; then
        source "${WS_DIR}/mujoco_env.sh"
    fi

    echo "Environment loaded ✅"
    echo "Launching backend=${BACKEND} robot=${ROBOT_TYPE} contact_source=${CONTACT_SOURCE} debug_state_logging=${DEBUG_STATE_LOGGING} rviz_auto=${RVIZ_AUTO} gui_auto=${GUI_AUTO} rviz_source=${RVIZ_SOURCE} ros_domain_id=${ROS_DOMAIN_ID}..."

    ./tools/run_tmux.sh "${ROBOT_TYPE}" "${BACKEND}" "${CONTACT_SOURCE}" "${DEBUG_STATE_LOGGING}" "${RVIZ_AUTO}" "${GUI_AUTO}" "${RVIZ_SOURCE}"

else
    echo "Running on host system"
    echo "Starting Docker container..."

    xhost +local:docker >/dev/null 2>&1 || true

    docker compose up -d

    echo "Attaching to container..."

    docker exec -it -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" $(docker compose ps -q quad_ocs2) ./run.sh "${ROBOT_TYPE}" "${BACKEND}" "${CONTACT_SOURCE}" "${DEBUG_STATE_LOGGING}" "${RVIZ_AUTO}" "${GUI_AUTO}" "${RVIZ_SOURCE}"
fi
