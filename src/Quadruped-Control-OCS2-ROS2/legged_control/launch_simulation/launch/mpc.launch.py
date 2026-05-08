import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _resolve_task_file(user_command_share, robot_type, task_file_name):
    config_dir = os.path.join(user_command_share, 'config', robot_type)
    candidate_name = task_file_name if task_file_name else 'task.info'
    candidate = candidate_name if os.path.isabs(candidate_name) else os.path.join(config_dir, candidate_name)
    if not os.path.exists(candidate):
        raise RuntimeError(f'Requested task file does not exist: {candidate}')
    return candidate


def _create_nodes(context):
    robot_type = LaunchConfiguration('robot_type').perform(context)
    task_file_name = LaunchConfiguration('task_file_name').perform(context)
    mujoco_share = get_package_share_directory('mujoco_simulator')
    user_command_share = get_package_share_directory('user_command')

    urdf_file = os.path.join(mujoco_share, 'models', robot_type, 'urdf', 'robot.urdf')
    simulator_file = os.path.join(user_command_share, 'config', robot_type, 'simulation.info')
    reference_file = os.path.join(user_command_share, 'config', robot_type, 'reference.info')
    task_file = _resolve_task_file(user_command_share, robot_type, task_file_name)

    return [
        Node(
            package='motion_control',
            executable='legged_robot_sqp_mpc',
            name='legged_robot_sqp_mpc',
            output='screen',
            parameters=[
                {'robotName': 'legged_robot'},
                {'controlType': 'mpc'},
                {'urdfFile': urdf_file},
                {'taskFile': task_file},
                {'referenceFile': reference_file},
                {'simulatorFile': simulator_file}
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type',
            default_value='quad_mini_tuned'
        ),
        DeclareLaunchArgument(
            'task_file_name',
            default_value='task.info'
        ),
        OpaqueFunction(function=_create_nodes),
    ])
