import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('one_leg_pinocchio_control')
    default_config = os.path.join(pkg_share, 'config', 'one_leg_inverse_dynamics.yaml')
    default_urdf = os.path.join(pkg_share, 'urdf', 'bacak_test_description.urdf')
    default_rviz = os.path.join(pkg_share, 'rviz', 'one_leg.rviz')

    config_file = LaunchConfiguration('config_file')
    urdf_path = LaunchConfiguration('urdf_path')
    rviz_config = LaunchConfiguration('rviz_config')
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML config for the one-leg RViz state mapper.',
        ),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=default_urdf,
            description='URDF to display in RViz.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='RViz config file.',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='one_leg_map_to_base',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        ),
        Node(
            package='one_leg_pinocchio_control',
            executable='one_leg_state_to_model_joint_state.py',
            name='one_leg_state_to_model_joint_state',
            output='screen',
            parameters=[
                config_file,
                {
                    'input_topic': 'htdw_joint_state',
                    'output_topic': 'one_leg_model_joint_states',
                }
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='one_leg_robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'publish_frequency': 60.0,
                }
            ],
            remappings=[
                ('joint_states', 'one_leg_model_joint_states'),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='one_leg_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
