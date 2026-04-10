import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('one_leg_pinocchio_control')
    default_config = os.path.join(pkg_share, 'config', 'one_leg_inverse_dynamics.yaml')
    default_urdf = os.path.join(pkg_share, 'urdf', 'bacak_test_description.urdf')

    config_file = LaunchConfiguration('config_file')
    urdf_path = LaunchConfiguration('urdf_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='YAML config for the isolated one-leg inverse dynamics controller.',
        ),
        DeclareLaunchArgument(
            'urdf_path',
            default_value=default_urdf,
            description='URDF used by Pinocchio for inverse dynamics.',
        ),
        Node(
            package='one_leg_pinocchio_control',
            executable='one_leg_inverse_dynamics_node',
            name='one_leg_inverse_dynamics_node',
            output='screen',
            parameters=[
                config_file,
                {'urdf_path': urdf_path},
            ],
        ),
    ])
