import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    robot_type = LaunchConfiguration('robot_type')

    urdfFile = PathJoinSubstitution([
        get_package_share_directory('mujoco_simulator'),
        'models',
        robot_type,
        'urdf',
        'robot.urdf'
    ])

    simulatorFile = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'simulation.info'
    ])

    referenceFile = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'reference.info'
    ])

    taskFile = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'task.info'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_type',
            default_value='b1'
        ),

        Node(
            package='motion_control',
            executable='legged_robot_sqp_mpc',
            name='legged_robot_sqp_mpc',
            output='screen',
            parameters=[
                {'urdfFile': urdfFile},
                {'taskFile': taskFile},
                {'referenceFile': referenceFile},
                {'simulatorFile': simulatorFile}
            ]
        )
    ])