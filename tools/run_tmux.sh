#!/usr/bin/env bash
set -e

SESSION=quad
WS=/workspaces/quad_ocs2_ws

# robot type argument (default b1)
ROBOT_TYPE=${1:-b1}

echo "Launching robot: $ROBOT_TYPE"

tmux new-session -d -s $SESSION

tmux split-window -h -t $SESSION
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.1

tmux select-layout -t $SESSION tiled


# -------------------------
# Mujoco simulator
# -------------------------
tmux respawn-pane -k -t $SESSION:0.0 \
"bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
ros2 launch launch_simulation mujoco.launch.py robot_type:=$ROBOT_TYPE
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
sleep 5
ros2 launch launch_simulation mpc.launch.py robot_type:=$ROBOT_TYPE
'"


# -------------------------
# Debug terminal
# -------------------------
tmux respawn-pane -k -t $SESSION:0.3 \
"bash -lc '
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo Debug terminal ready
bash
'"

# Focus the interactive user-command pane when attaching.
tmux select-pane -t $SESSION:0.1

tmux attach-session -t $SESSION
