import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    robot_type = LaunchConfiguration('robot_type')
    control_type = LaunchConfiguration('control_type')
    mujoco_timestep = LaunchConfiguration('mujoco_timestep')
    mujoco_control_frequency = LaunchConfiguration('mujoco_control_frequency')
    mujoco_base_kp = LaunchConfiguration('mujoco_base_kp')
    mujoco_base_kd = LaunchConfiguration('mujoco_base_kd')

    xmlFile = PathJoinSubstitution([
        get_package_share_directory('mujoco_simulator'),
        'models',
        robot_type,
        'urdf',
        'robot.xml'
    ])

    simulatorFile = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'simulation.info'
    ])

    rlFile = PathJoinSubstitution([
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
        DeclareLaunchArgument('control_type', default_value='mpc'),
        DeclareLaunchArgument('mujoco_timestep', default_value='0.0'),
        DeclareLaunchArgument('mujoco_control_frequency', default_value='0.0'),
        DeclareLaunchArgument('mujoco_base_kp', default_value='0.0'),
        DeclareLaunchArgument('mujoco_base_kd', default_value='0.0'),

        Node(
            package='mujoco_simulator',
            executable='mujoco_simulator',
            name='mujoco_simulator',
            output='screen',
            parameters=[
                {'xmlFile': xmlFile},
                {'simulatorFile': simulatorFile},
                {'rlFile': rlFile},
                {'controlType': control_type},
                {'mujocoTimestep': ParameterValue(mujoco_timestep, value_type=float)},
                {'mujocoControlFrequency': ParameterValue(mujoco_control_frequency, value_type=float)},
                {'mujocoBaseKp': ParameterValue(mujoco_base_kp, value_type=float)},
                {'mujocoBaseKd': ParameterValue(mujoco_base_kd, value_type=float)},
            ]
        )
    ])
