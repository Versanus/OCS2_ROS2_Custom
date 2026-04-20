#!/usr/bin/env bash

set -e

SESSION="quad_viewer"
WS=/workspaces/quad_ocs2_ws
VIEWER_MODE="${1:-sim}"
ROBOT_TYPE="${2:-quad_mini_real}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ROS_DOMAIN_ID

case "${VIEWER_MODE}" in
  sim|hardware) ;;
  real)
    VIEWER_MODE="hardware"
    ;;
  *)
    echo "Invalid viewer mode: ${VIEWER_MODE}. Use 'sim' or 'hardware'."
    exit 1
    ;;
esac

TASK_FILE="$WS/install/user_command/share/user_command/config/$ROBOT_TYPE/task.info"
STATE_ESTIMATE=false

if [ -f "$TASK_FILE" ]; then
  STATE_ESTIMATE=$(awk '$1 == "stateEstimate" {print tolower($2); exit}' "$TASK_FILE")
fi

WAIT_TOPICS=()
USE_HARDWARE_GUI=false

if [ "$VIEWER_MODE" = "hardware" ]; then
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=hardware odom_source:=topic input_joint_state_topic:=htdw_joint_state output_joint_state_topic:=rviz_joint_states odom_topic:=odom path_topic:=rviz_hardware_odom_path"
  WAIT_TOPICS=(/htdw_joint_state /odom)
  USE_HARDWARE_GUI=true
elif [ "$STATE_ESTIMATE" = "true" ]; then
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=sensor odom_source:=topic sensor_input_topic:=simulator_sensor_data output_joint_state_topic:=rviz_joint_states odom_topic:=odom path_topic:=rviz_odom_path"
  WAIT_TOPICS=(/simulator_sensor_data /odom)
else
  RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=state odom_source:=state state_input_topic:=simulator_state_data output_joint_state_topic:=rviz_joint_states odom_topic:=rviz_odom path_topic:=rviz_odom_path"
  WAIT_TOPICS=(/simulator_state_data)
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n debug
tmux set-option -t "$SESSION" remain-on-exit on
tmux set-environment -t "$SESSION" ROS_DOMAIN_ID "$ROS_DOMAIN_ID"
tmux bind-key -T root C-c kill-session -t "$SESSION"

tmux respawn-pane -k -t "$SESSION:0" "bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo Viewer debug shell ready
echo Mode: $VIEWER_MODE
echo Robot: $ROBOT_TYPE
echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID
echo
echo Useful checks:
echo   ros2 topic list
echo   ros2 topic hz /mpc_compute_time_ms
if [ \"$VIEWER_MODE\" = \"sim\" ]; then
  echo   ros2 topic hz /simulator_sensor_data
  echo   ros2 topic hz /simulator_state_data
else
  echo   ros2 topic hz /htdw_joint_state
  echo   ros2 topic hz /odom
fi
echo
bash
'"

WAIT_SCRIPT=""
for topic in "${WAIT_TOPICS[@]}"; do
  WAIT_SCRIPT+="echo Waiting for ${topic}...; "
  WAIT_SCRIPT+="until ros2 topic type ${topic} >/dev/null 2>&1; do sleep 1; done; "
  WAIT_SCRIPT+="echo ${topic} is available.; "
done

tmux new-window -t "$SESSION" -n rviz
tmux respawn-pane -k -t "$SESSION:1" "bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
$WAIT_SCRIPT
echo Launching RViz...
$RVIZ_COMMAND
'"

tmux new-window -t "$SESSION" -n timing
tmux respawn-pane -k -t "$SESSION:2" "bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo Waiting for /mpc_compute_time_ms...
until ros2 topic type /mpc_compute_time_ms >/dev/null 2>&1; do
  sleep 1
done
echo /mpc_compute_time_ms is available. Echoing latest MPC compute times in ms.
ros2 topic echo /mpc_compute_time_ms
'"

if [ "$USE_HARDWARE_GUI" = true ]; then
  tmux new-window -t "$SESSION" -n gui
  tmux respawn-pane -k -t "$SESSION:3" "bash -lc '
  source /opt/ros/humble/setup.bash
  source $WS/install/setup.bash
  echo Waiting for /htdw_joint_state...
  until ros2 topic type /htdw_joint_state >/dev/null 2>&1; do
    sleep 1
  done
  echo Launching hardware status GUI...
  ros2 run hardware_interface gui_status.py
  '"
fi

tmux select-window -t "$SESSION:0"
tmux attach-session -t "$SESSION"
