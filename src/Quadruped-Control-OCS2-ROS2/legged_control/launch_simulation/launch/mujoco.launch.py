import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    robot_type = LaunchConfiguration('robot_type')

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

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_type',
            default_value='b1'
        ),

        Node(
            package='mujoco_simulator',
            executable='mujoco_simulator',
            name='mujoco_simulator',
            output='screen',
            parameters=[
                {'xmlFile': xmlFile},
                {'simulatorFile': simulatorFile}
            ]
        )
    ])