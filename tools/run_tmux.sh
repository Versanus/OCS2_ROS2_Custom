#!/usr/bin/env bash
set -e

SESSION=quad
WS=/workspaces/quad_ocs2_ws

tmux new-session -d -s $SESSION

# create 4 panes first
tmux split-window -h -t $SESSION:0
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.1

tmux select-layout -t $SESSION tiled

# Pane 0 → Mujoco
tmux send-keys -t $SESSION:0.0 "
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
ros2 launch launch_simulation mujoco.launch.py robot_type:=b1
" C-m

# Pane 1 → User command
tmux send-keys -t $SESSION:0.1 "
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
ros2 launch launch_simulation user_command.launch.py robot_type:=b1
" C-m

# Pane 2 → MPC (with delay like TimerAction)
tmux send-keys -t $SESSION:0.2 "
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
sleep 5
ros2 launch launch_simulation mpc.launch.py robot_type:=b1
" C-m

# Pane 3 → debug
tmux send-keys -t $SESSION:0.3 "
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
echo 'Debug terminal ready'
bash
" C-m

tmux attach-session -t $SESSION
