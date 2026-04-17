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
    joint_feedback_source = LaunchConfiguration('joint_feedback_source')
    joint_feedback_topic = LaunchConfiguration('joint_feedback_topic')
    joint_state_topic = LaunchConfiguration('joint_state_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    hardware_command_topic = LaunchConfiguration('hardware_command_topic')

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

    urdf_file = PathJoinSubstitution([
        get_package_share_directory('mujoco_simulator'),
        'models',
        robot_type,
        'urdf',
        'robot.urdf'
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
        DeclareLaunchArgument('joint_feedback_source', default_value='joint_trajectory'),
        DeclareLaunchArgument('joint_feedback_topic', default_value='htdw_joint_cmd'),
        DeclareLaunchArgument('joint_state_topic', default_value='joint_states'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data'),
        DeclareLaunchArgument('odom_topic', default_value=''),
        DeclareLaunchArgument('hardware_command_topic', default_value='bridge_joint_command'),

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
                {'urdfFile': urdf_file},
                {'contactSource': contact_source},
                {'alwaysPublishStateTopic': True},
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
                {'urdfFile': urdf_file},
                {'contactSource': contact_source},
                {'alwaysPublishStateTopic': True},
                {'publishRateHz': ParameterValue(publish_rate_hz, value_type=float)},
                {'debugStateLogging': ParameterValue(debug_state_logging, value_type=bool)},
                {'jointFeedbackSource': joint_feedback_source},
                {'jointFeedbackTopic': joint_feedback_topic},
                {'jointStateTopic': joint_state_topic},
                {'imuTopic': imu_topic},
                {'odomTopic': odom_topic},
                {'hardwareCommandTopic': hardware_command_topic},
            ]
        ),
    ])
