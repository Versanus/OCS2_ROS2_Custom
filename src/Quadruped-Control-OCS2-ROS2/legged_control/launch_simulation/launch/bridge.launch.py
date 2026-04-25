import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _selected_xml_name(control_type, terrain):
    if terrain == 'rough':
        return 'robot_rough_RL.xml' if control_type == 'rl' else 'robot_rough.xml'
    if terrain == 'flat':
        return 'robot_RL_motor.xml' if control_type == 'rl' else 'robot.xml'
    raise RuntimeError("mujoco_terrain must be 'flat' or 'rough'.")


def _resolve_xml_file(mujoco_share, robot_type, control_type, terrain, xml_override):
    urdf_dir = os.path.join(mujoco_share, 'models', robot_type, 'urdf')
    if xml_override:
        xml_file = xml_override if os.path.isabs(xml_override) else os.path.join(urdf_dir, xml_override)
    else:
        xml_file = os.path.join(urdf_dir, _selected_xml_name(control_type, terrain))

    if not os.path.exists(xml_file):
        raise RuntimeError(
            f"Selected MuJoCo XML does not exist: {xml_file}. "
            f"control_type={control_type}, mujoco_terrain={terrain}."
        )
    return xml_file


def _create_nodes(context):
    robot_type_value = LaunchConfiguration('robot_type').perform(context)
    backend_value = LaunchConfiguration('backend').perform(context)
    control_type_value = LaunchConfiguration('control_type').perform(context).lower()
    terrain_value = LaunchConfiguration('mujoco_terrain').perform(context).lower()
    xml_override = LaunchConfiguration('mujoco_xml_file').perform(context)

    mujoco_share = get_package_share_directory('mujoco_simulator')
    user_command_share = get_package_share_directory('user_command')

    simulator_file = os.path.join(user_command_share, 'config', robot_type_value, 'simulation.info')
    urdf_file = os.path.join(mujoco_share, 'models', robot_type_value, 'urdf', 'robot.urdf')
    task_file = os.path.join(user_command_share, 'config', robot_type_value, 'task.info')
    rl_file = os.path.join(user_command_share, 'config', robot_type_value, 'rl.info')

    if backend_value == 'sim':
        xml_file = _resolve_xml_file(mujoco_share, robot_type_value, control_type_value, terrain_value, xml_override)
        return [
            Node(
                package='real_robot_bridge',
                executable='bridge_sim_node',
                name='bridge_sim_node',
                output='screen',
                parameters=[
                    {'xmlFile': xml_file},
                    {'simulatorFile': simulator_file},
                    {'taskFile': task_file},
                    {'rlFile': rl_file},
                    {'urdfFile': urdf_file},
                    {'contactSource': LaunchConfiguration('contact_source')},
                    {'controlType': LaunchConfiguration('control_type')},
                    {'alwaysPublishStateTopic': True},
                    {'publishRateHz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float)},
                    {'debugStateLogging': ParameterValue(LaunchConfiguration('debug_state_logging'), value_type=bool)},
                    {'mujocoTimestep': ParameterValue(LaunchConfiguration('mujoco_timestep'), value_type=float)},
                    {'mujocoControlFrequency': ParameterValue(LaunchConfiguration('mujoco_control_frequency'), value_type=float)},
                    {'mujocoBaseKp': ParameterValue(LaunchConfiguration('mujoco_base_kp'), value_type=float)},
                    {'mujocoBaseKd': ParameterValue(LaunchConfiguration('mujoco_base_kd'), value_type=float)},
                ]
            )
        ]

    if backend_value == 'real':
        return [
            Node(
                package='real_robot_bridge',
                executable='bridge_real_node',
                name='bridge_real_node',
                output='screen',
                parameters=[
                    {'taskFile': task_file},
                    {'urdfFile': urdf_file},
                    {'contactSource': LaunchConfiguration('contact_source')},
                    {'alwaysPublishStateTopic': True},
                    {'publishRateHz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float)},
                    {'debugStateLogging': ParameterValue(LaunchConfiguration('debug_state_logging'), value_type=bool)},
                    {'jointFeedbackSource': LaunchConfiguration('joint_feedback_source')},
                    {'jointFeedbackTopic': LaunchConfiguration('joint_feedback_topic')},
                    {'jointStateTopic': LaunchConfiguration('joint_state_topic')},
                    {'imuTopic': LaunchConfiguration('imu_topic')},
                    {'odomTopic': LaunchConfiguration('odom_topic')},
                    {'hardwareCommandTopic': LaunchConfiguration('hardware_command_topic')},
                ]
            )
        ]

    raise RuntimeError("backend must be 'sim' or 'real'.")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='b1'),
        DeclareLaunchArgument('backend', default_value='sim'),
        DeclareLaunchArgument('contact_source', default_value='mujoco'),
        DeclareLaunchArgument('publish_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('debug_state_logging', default_value='false'),
        DeclareLaunchArgument('control_type', default_value='mpc'),
        DeclareLaunchArgument('mujoco_terrain', default_value='flat'),
        DeclareLaunchArgument('mujoco_xml_file', default_value=''),
        DeclareLaunchArgument('joint_feedback_source', default_value='joint_trajectory'),
        DeclareLaunchArgument('joint_feedback_topic', default_value='htdw_joint_cmd'),
        DeclareLaunchArgument('joint_state_topic', default_value='joint_states'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data'),
        DeclareLaunchArgument('odom_topic', default_value=''),
        DeclareLaunchArgument('hardware_command_topic', default_value='bridge_joint_command'),
        DeclareLaunchArgument('mujoco_timestep', default_value='0.0'),
        DeclareLaunchArgument('mujoco_control_frequency', default_value='0.0'),
        DeclareLaunchArgument('mujoco_base_kp', default_value='0.0'),
        DeclareLaunchArgument('mujoco_base_kd', default_value='0.0'),
        OpaqueFunction(function=_create_nodes),
    ])
