cd ~/quad_ocs2_ws

rm -rf build/motion_control
rm -rf install/motion_control


source /opt/ros/humble/setup.bash
source install/setup.bash
source mujoco_env.sh

colcon build --packages-select motion_control --cmake-clean-cache

source install/local_setup.sh