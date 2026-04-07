import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_type = LaunchConfiguration('robot_type')
    backend = LaunchConfiguration('backend')
    contact_source = LaunchConfiguration('contact_source')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    debug_state_logging = LaunchConfiguration('debug_state_logging')

    xml_file = PathJoinSubstitution([
        get_package_share_directory('mujoco_simulator'),
        'models',
        robot_type,
        'urdf',
        'robot.xml'
    ])

    simulator_file = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'simulation.info'
    ])

    task_file = PathJoinSubstitution([
        get_package_share_directory('user_command'),
        'config',
        robot_type,
        'task.info'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='b1'),
        DeclareLaunchArgument('backend', default_value='sim'),
        DeclareLaunchArgument('contact_source', default_value='mujoco'),
        DeclareLaunchArgument('publish_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('debug_state_logging', default_value='false'),

        Node(
            package='real_robot_bridge',
            executable='bridge_sim_node',
            name='bridge_sim_node',
            output='screen',
            condition=IfCondition(PythonExpression(["'", backend, "' == 'sim'"])),
            parameters=[
                {'xmlFile': xml_file},
                {'simulatorFile': simulator_file},
                {'taskFile': task_file},
                {'contactSource': contact_source},
                {'publishRateHz': ParameterValue(publish_rate_hz, value_type=float)},
                {'debugStateLogging': ParameterValue(debug_state_logging, value_type=bool)},
            ]
        ),

        Node(
            package='real_robot_bridge',
            executable='bridge_real_node',
            name='bridge_real_node',
            output='screen',
            condition=IfCondition(PythonExpression(["'", backend, "' == 'real'"])),
            parameters=[
                {'taskFile': task_file},
                {'contactSource': contact_source},
                {'publishRateHz': ParameterValue(publish_rate_hz, value_type=float)},
                {'debugStateLogging': ParameterValue(debug_state_logging, value_type=bool)},
            ]
        ),
    ])
