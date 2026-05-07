#!/usr/bin/env bash
#./run.sh quad_mini sim
#./run.sh quad_mini sim debug
#./run.sh quad_mini sim mujoco debug
#./run.sh quad_mini sim mujoco debug rviz gui
#./run.sh quad_mini sim mujoco nodebug norviz nogui
#./run.sh quad_mini_real real estimated debug rviz gui
#./run.sh quad_mini_real sim estimated nodebug norviz nogui auto rl
#./run.sh quad_mini_real sim estimated nodebug norviz nogui auto rl rough
#./run.sh quad_mini_tuned gazebo estimated debug rviz nogui auto mpc flat
#./run.sh quad_mini_tuned gazebo_headless estimated debug rviz nogui auto mpc flat

set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_TYPE="${1:-quad_mini}"
BACKEND="${2:-sim}"
CONTACT_SOURCE="${3:-}"
DEBUG_STATE_LOGGING="${4:-}"
RVIZ_AUTO="${5:-}"
GUI_AUTO="${6:-}"
RVIZ_SOURCE="${7:-auto}"
CONTROL_TYPE="${8:-mpc}"
MUJOCO_TERRAIN="${9:-flat}"
GAZEBO_HEADLESS="${10:-}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ROS_DOMAIN_ID
HOST_UID="${HOST_UID:-$(id -u)}"
HOST_GID="${HOST_GID:-$(id -g)}"
if [ -z "${INPUT_GID:-}" ]; then
    if getent group input >/dev/null 2>&1; then
        INPUT_GID="$(getent group input | cut -d: -f3)"
    elif [ -e /dev/input ]; then
        INPUT_GID="$(stat -c '%g' /dev/input)"
    else
        INPUT_GID="${HOST_GID}"
    fi
fi
export HOST_UID HOST_GID INPUT_GID

if [ "${CONTACT_SOURCE}" = "debug" ] || [ "${CONTACT_SOURCE}" = "nodebug" ] || \
   [ "${CONTACT_SOURCE}" = "true" ] || [ "${CONTACT_SOURCE}" = "false" ]; then
    DEBUG_STATE_LOGGING="${CONTACT_SOURCE}"
    CONTACT_SOURCE=""
fi

if [ "${BACKEND}" = "gazebo_headless" ]; then
    BACKEND="gazebo"
    if [ -z "${GAZEBO_HEADLESS}" ]; then
        GAZEBO_HEADLESS="true"
    fi
fi

if [ -z "${CONTACT_SOURCE}" ]; then
    if [ "${BACKEND}" = "real" ] || [ "${BACKEND}" = "gazebo" ]; then
        CONTACT_SOURCE="estimated"
    else
        CONTACT_SOURCE="mujoco"
    fi
fi

case "${BACKEND}" in
    sim|real|gazebo) ;;
    *)
        echo "Invalid backend: ${BACKEND}. Use 'sim', 'real', or 'gazebo'."
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

case "${CONTROL_TYPE}" in
    mpc|rl)
        ;;
    *)
        echo "Invalid control type: ${CONTROL_TYPE}. Use 'mpc' or 'rl'."
        exit 1
        ;;
esac

case "${MUJOCO_TERRAIN}" in
    flat|rough)
        ;;
    *)
        echo "Invalid MuJoCo terrain: ${MUJOCO_TERRAIN}. Use 'flat' or 'rough'."
        exit 1
        ;;
esac

case "${GAZEBO_HEADLESS}" in
    ""|window|gui|false)
        GAZEBO_HEADLESS="false"
        ;;
    headless|true)
        GAZEBO_HEADLESS="true"
        ;;
    *)
        echo "Invalid Gazebo headless flag: ${GAZEBO_HEADLESS}. Use 'headless', 'window', 'true', or 'false'."
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
    if [ "${BACKEND}" = "sim" ]; then
        source "${WS_DIR}/mujoco_env.sh"
    fi

    echo "Environment loaded ✅"
    echo "Launching backend=${BACKEND} robot=${ROBOT_TYPE} contact_source=${CONTACT_SOURCE} debug_state_logging=${DEBUG_STATE_LOGGING} rviz_auto=${RVIZ_AUTO} gui_auto=${GUI_AUTO} rviz_source=${RVIZ_SOURCE} control_type=${CONTROL_TYPE} mujoco_terrain=${MUJOCO_TERRAIN} gazebo_headless=${GAZEBO_HEADLESS} ros_domain_id=${ROS_DOMAIN_ID}..."

    ./tools/run_tmux.sh "${ROBOT_TYPE}" "${BACKEND}" "${CONTACT_SOURCE}" "${DEBUG_STATE_LOGGING}" "${RVIZ_AUTO}" "${GUI_AUTO}" "${RVIZ_SOURCE}" "${CONTROL_TYPE}" "${MUJOCO_TERRAIN}" "${GAZEBO_HEADLESS}"

else
    echo "Running on host system"
    echo "Starting Docker container..."

    xhost +local:docker >/dev/null 2>&1 || true

    cleanup_done=false
    cleanup_container() {
        if [ "${cleanup_done}" = true ]; then
            return
        fi
        cleanup_done=true
        docker compose down --timeout 1 >/dev/null 2>&1 || true
    }

    handle_interrupt() {
        cleanup_container
        exit 130
    }

    trap cleanup_container EXIT
    trap handle_interrupt INT TERM

    docker compose up -d

    echo "Attaching to container..."

    docker exec -it \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
        "$(docker compose ps -q quad_ocs2)" \
        ./run.sh "${ROBOT_TYPE}" "${BACKEND}" "${CONTACT_SOURCE}" "${DEBUG_STATE_LOGGING}" "${RVIZ_AUTO}" "${GUI_AUTO}" "${RVIZ_SOURCE}" "${CONTROL_TYPE}" "${MUJOCO_TERRAIN}" "${GAZEBO_HEADLESS}"
fi
