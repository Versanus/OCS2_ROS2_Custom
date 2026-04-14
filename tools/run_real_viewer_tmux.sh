#!/usr/bin/env bash

set -e

SESSION="quad_viewer"
WS=/workspaces/quad_ocs2_ws
ROBOT_TYPE="${1:-quad_mini_real}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
export ROS_DOMAIN_ID

RVIZ_COMMAND="ros2 launch hardware_interface kalman_state_rviz.launch.py robot_type:=$ROBOT_TYPE joint_source:=hardware odom_source:=topic input_joint_state_topic:=htdw_joint_state output_joint_state_topic:=rviz_joint_states odom_topic:=odom path_topic:=rviz_hardware_odom_path"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n debug
tmux set-environment -t "$SESSION" ROS_DOMAIN_ID "$ROS_DOMAIN_ID"

tmux respawn-pane -k -t "$SESSION:0" "bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo Real hardware viewer debug shell ready
echo Waiting for topics from the robot on ROS_DOMAIN_ID=$ROS_DOMAIN_ID
echo Useful checks:
echo   ros2 topic list
echo   ros2 topic hz /htdw_joint_state
echo   ros2 topic hz /odom
bash
'"

tmux new-window -t "$SESSION" -n rviz
tmux respawn-pane -k -t "$SESSION:1" "bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo Waiting for /htdw_joint_state...
until ros2 topic type /htdw_joint_state >/dev/null 2>&1; do
  sleep 1
done
echo /htdw_joint_state is available.
echo Waiting for /odom...
until ros2 topic type /odom >/dev/null 2>&1; do
  sleep 1
done
echo /odom is available. Launching RViz...
$RVIZ_COMMAND
'"

tmux new-window -t "$SESSION" -n gui
tmux respawn-pane -k -t "$SESSION:2" "bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo Waiting for /htdw_joint_state...
until ros2 topic type /htdw_joint_state >/dev/null 2>&1; do
  sleep 1
done
echo Launching hardware status GUI...
ros2 run hardware_interface gui_status.py
'"

tmux select-window -t "$SESSION:0"
tmux attach-session -t "$SESSION"
