#!/usr/bin/env bash
set -e
#./run.sh quad_mini sim mujoco debug rviz gui
#./run.sh quad_mini sim mujoco nodebug norviz nogui

SESSION=quad
WS=/workspaces/quad_ocs2_ws

# robot type argument
ROBOT_TYPE=${1:-quad_mini}
BACKEND=${2:-sim}
CONTACT_SOURCE=${3:-}
DEBUG_STATE_LOGGING=${4:-false}
RVIZ_AUTO=${5:-true}
GUI_AUTO=${6:-true}
RVIZ_SOURCE=${7:-auto}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-23}
export ROS_DOMAIN_ID

if [ -z "$CONTACT_SOURCE" ]; then
  if [ "$BACKEND" = "real" ]; then
    CONTACT_SOURCE=estimated
  else
    CONTACT_SOURCE=mujoco
  fi
fi

case "$BACKEND" in
  sim|real) ;;
  *)
    echo "Invalid backend: $BACKEND. Use 'sim' or 'real'."
    exit 1
    ;;
esac

case "$CONTACT_SOURCE" in
  mujoco|estimated) ;;
  *)
    echo "Invalid contact source: $CONTACT_SOURCE. Use 'mujoco' or 'estimated'."
    exit 1
    ;;
esac

case "$DEBUG_STATE_LOGGING" in
  debug|true)
    DEBUG_STATE_LOGGING=true
    ;;
  nodebug|false|"")
    DEBUG_STATE_LOGGING=false
    ;;
  *)
    echo "Invalid debug flag: $DEBUG_STATE_LOGGING. Use 'debug', 'nodebug', 'true', or 'false'."
    exit 1
    ;;
esac

case "$RVIZ_AUTO" in
  rviz|true|"")
    RVIZ_AUTO=true
    ;;
  norviz|false)
    RVIZ_AUTO=false
    ;;
  *)
    echo "Invalid RViz flag: $RVIZ_AUTO. Use 'rviz', 'norviz', 'true', or 'false'."
    exit 1
    ;;
esac

case "$GUI_AUTO" in
  gui|true|"")
    GUI_AUTO=true
    ;;
  nogui|false)
    GUI_AUTO=false
    ;;
  *)
    echo "Invalid GUI flag: $GUI_AUTO. Use 'gui', 'nogui', 'true', or 'false'."
    exit 1
    ;;
esac

case "$RVIZ_SOURCE" in
  auto|sim|hardware) ;;
  *)
    echo "Invalid RViz source: $RVIZ_SOURCE. Use 'auto', 'sim', or 'hardware'."
    exit 1
    ;;
esac

echo "Launching robot: $ROBOT_TYPE"
echo "Backend: $BACKEND"
echo "Contact source: $CONTACT_SOURCE"
echo "Bridge debug state logging: $DEBUG_STATE_LOGGING"
echo "Auto launch RViz: $RVIZ_AUTO"
echo "Auto launch GUI: $GUI_AUTO"
echo "RViz source: $RVIZ_SOURCE"
echo "ROS domain ID: $ROS_DOMAIN_ID"

tmux new-session -d -s $SESSION
tmux set-environment -t $SESSION ROS_DOMAIN_ID "$ROS_DOMAIN_ID"

tmux split-window -h -t $SESSION
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.1

tmux select-layout -t $SESSION tiled

TASK_FILE="$WS/install/user_command/share/user_command/config/$ROBOT_TYPE/task.info"
STATE_ESTIMATE=false

if [ -f "$TASK_FILE" ]; then
  STATE_ESTIMATE=$(awk '$1 == "stateEstimate" {print tolower($2); exit}' "$TASK_FILE")
fi

# Always use the moving-base RViz launcher. The source behind it changes based on backend/data availability.
if [ "$RVIZ_SOURCE" = "hardware" ]; then
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=hardware odom_source:=topic input_joint_state_topic:=htdw_joint_state output_joint_state_topic:=rviz_joint_states odom_topic:=rviz_hardware_odom path_topic:=rviz_hardware_odom_path"
elif [ "$RVIZ_SOURCE" = "sim" ] && [ "$STATE_ESTIMATE" = "true" ]; then
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=sensor odom_source:=topic sensor_input_topic:=simulator_sensor_data output_joint_state_topic:=rviz_joint_states odom_topic:=odom path_topic:=rviz_odom_path"
elif [ "$RVIZ_SOURCE" = "sim" ]; then
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=state odom_source:=state state_input_topic:=simulator_state_data output_joint_state_topic:=rviz_joint_states odom_topic:=rviz_odom path_topic:=rviz_odom_path"
elif [ "$BACKEND" = "sim" ] && [ "$STATE_ESTIMATE" = "true" ]; then
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=sensor odom_source:=topic sensor_input_topic:=simulator_sensor_data output_joint_state_topic:=rviz_joint_states odom_topic:=odom path_topic:=rviz_odom_path"
elif [ "$BACKEND" = "sim" ]; then
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=state odom_source:=state state_input_topic:=simulator_state_data output_joint_state_topic:=rviz_joint_states odom_topic:=rviz_odom path_topic:=rviz_odom_path"
elif [ "$STATE_ESTIMATE" = "true" ]; then
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=sensor odom_source:=topic sensor_input_topic:=simulator_sensor_data output_joint_state_topic:=rviz_joint_states odom_topic:=odom path_topic:=rviz_odom_path"
else
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=state odom_source:=state state_input_topic:=simulator_state_data output_joint_state_topic:=rviz_joint_states odom_topic:=rviz_odom path_topic:=rviz_odom_path"
fi


# -------------------------
# Shared bridge backend
# -------------------------
tmux respawn-pane -k -t $SESSION:0.0 \
"bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
if [ \"$BACKEND\" != \"real\" ] && [ -f $WS/mujoco_env.sh ]; then
  source $WS/mujoco_env.sh
fi
ros2 launch launch_simulation bridge.launch.py robot_type:=$ROBOT_TYPE backend:=$BACKEND contact_source:=$CONTACT_SOURCE debug_state_logging:=$DEBUG_STATE_LOGGING
'"


# -------------------------
# User command (interactive)
# -------------------------
tmux respawn-pane -k -t $SESSION:0.1 \
"bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash

REF=\$(ros2 pkg prefix user_command)/share/user_command/config/$ROBOT_TYPE/reference.info
GAIT=\$(ros2 pkg prefix user_command)/share/user_command/config/$ROBOT_TYPE/gait.info

ros2 run user_command user_command_node \
 --ros-args \
 -p referenceFile:=\$REF \
 -p gaitCommandFile:=\$GAIT
'"


# -------------------------
# MPC controller
# -------------------------
tmux respawn-pane -k -t $SESSION:0.2 \
"bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo Waiting for /start_control service...
until ros2 service type /start_control >/dev/null 2>&1; do
  sleep 1
done
echo /start_control is ready. Launching MPC...
ros2 launch launch_simulation mpc.launch.py robot_type:=$ROBOT_TYPE
'"


# -------------------------
# Debug terminal
# -------------------------
tmux respawn-pane -k -t $SESSION:0.3 \
"bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
if [ \"$BACKEND\" = \"real\" ]; then
  ros2 run hardware_interface odom_relay.py --ros-args -p input_topic:=odom -p output_topic:=rviz_hardware_odom >/tmp/rviz_hardware_odom.log 2>&1 &
fi
echo Debug terminal ready
bash
'"

if [ "$RVIZ_AUTO" = true ]; then
  tmux new-window -t $SESSION -n rviz
  tmux respawn-pane -k -t $SESSION:1 \
  "bash -lc '
  source /opt/ros/humble/setup.bash
  source $WS/install/setup.bash
  if [ \"$BACKEND\" != \"real\" ] && [ -f $WS/mujoco_env.sh ]; then
    source $WS/mujoco_env.sh
  fi
  $RVIZ_COMMAND
  '"
fi

if [ "$GUI_AUTO" = true ]; then
  tmux new-window -t $SESSION -n gui
  tmux respawn-pane -k -t $SESSION:2 \
  "bash -lc '
  source /opt/ros/humble/setup.bash
  source $WS/install/setup.bash
  ros2 run hardware_interface gui_status.py
  '"
fi

# Focus the interactive user-command pane when attaching.
tmux select-window -t $SESSION:0
tmux select-pane -t $SESSION:0.1

tmux attach-session -t $SESSION
