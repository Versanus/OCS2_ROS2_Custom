import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    robot_name = LaunchConfiguration('robot_name')
    control_type = LaunchConfiguration('control_type')
    rl_config_file = LaunchConfiguration('rl_config_file')

    urdf_file = PathJoinSubstitution([
        get_package_share_directory('mujoco_simulator'),
        'models',
        robot_type,
        'urdf',
        'robot.urdf'
    ])

    simulator_file = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'simulation.info'
    ])

    reference_file = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'reference.info'
    ])

    task_file = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'task.info'
    ])

    default_rl_config_file = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'rl.info'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type',
            default_value='b1'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='legged_robot'
        ),
        DeclareLaunchArgument(
            'control_type',
            default_value='mpc'
        ),
        DeclareLaunchArgument(
            'rl_config_file',
            default_value=default_rl_config_file
        ),
        Node(
            package='motion_control',
            executable='legged_robot_controller',
            name='legged_robot_controller',
            output='screen',
            parameters=[
                {'robotName': robot_name},
                {'controlType': control_type},
                {'urdfFile': urdf_file},
                {'taskFile': task_file},
                {'referenceFile': reference_file},
                {'simulatorFile': simulator_file},
                {'rlConfigFile': rl_config_file}
            ]
        )
    ])
