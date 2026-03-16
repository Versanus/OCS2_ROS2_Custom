import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    robot_type = LaunchConfiguration('robot_type')

    gaitCommandFile = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'gait.info'
    ])

    referenceFile = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'reference.info'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_type',
            default_value='b1'
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'user_command', 'user_command_node',
                '--ros-args',
                '-p', ['referenceFile:=', referenceFile],
                '-p', ['gaitCommandFile:=', gaitCommandFile]
            ],
            output='screen'
        )
    ])