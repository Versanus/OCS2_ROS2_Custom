import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _create_nodes(context):
    mujoco_share = get_package_share_directory('mujoco_simulator')
    robot_type = LaunchConfiguration('robot_type').perform(context)
    input_topic = LaunchConfiguration('input_topic').perform(context)
    output_topic = LaunchConfiguration('output_topic').perform(context)

    urdf_file = os.path.join(mujoco_share, 'models', robot_type, 'urdf', 'robot.urdf')
    rviz_config_file = os.path.join(mujoco_share, 'config', 'visualize_urdf.rviz')
    mesh_dir = os.path.join(mujoco_share, 'models', robot_type, 'meshes')

    with open(urdf_file, 'r', encoding='utf-8') as urdf_handle:
        robot_description = urdf_handle.read()
    robot_description = robot_description.replace(
        'filename="meshes/',
        f'filename="file://{mesh_dir}/'
    )

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
        Node(
            package='hardware_inter',
            executable='mujoco_to_joint_state.py',
            name='mujoco_to_joint_state',
            output='screen',
            parameters=[
                {'input_topic': input_topic},
                {'output_topic': output_topic},
                {'publish_velocity': True},
                {'publish_effort': False},
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[
                        {
                            'publish_frequency': 100.0,
                            'use_tf_static': True,
                            'robot_description': robot_description,
                        }
                    ],
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_link_alias_publisher',
                    output='screen',
                    arguments=[
                        '--x', '0', '--y', '0', '--z', '0',
                        '--yaw', '0', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'base',
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='quad_mini_tuned'),
        DeclareLaunchArgument('input_topic', default_value='simulator_state_data'),
        DeclareLaunchArgument('output_topic', default_value='joint_states'),
        OpaqueFunction(function=_create_nodes),
    ])
