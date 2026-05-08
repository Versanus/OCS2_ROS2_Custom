# Quadruped-Control-OCS2-ROS2

This directory contains the ROS 2 packages used by the top-level Quad Mini workspace.

Use the repository-root README and scripts as the maintained workflow:

```bash
cd /workspaces/quad_ocs2_ws
./build.sh
./run.sh quad_mini_tuned sim mujoco debug rviz gui auto mpc flat
```

External repositories under this directory are pinned by the workspace `.gitmodules` file. Do not clone or update `pinocchio`, `hpp-fcl`, `ocs2_robotic_assets`, or `plane_segmentation_ros2` by hand for normal development; use:

```bash
git submodule update --init --recursive
```

Maintained robot configuration directories:

- `legged_control/user_command/config/quad_mini_tuned`
- `legged_control/user_command/config/quad_mini_real`

Maintained robot model directories:

- `legged_control/mujoco_simulator/models/quad_mini_tuned`
- `legged_control/mujoco_simulator/models/quad_mini_real`
