import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _selected_xml_name(control_type, terrain):
    if terrain == 'rough':
        return 'robot_rough_RL_motor.xml' if control_type == 'rl' else 'robot_rough.xml'
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


def _resolve_urdf_file(mujoco_share, robot_type, urdf_name):
    urdf_dir = os.path.join(mujoco_share, 'models', robot_type, 'urdf')
    if urdf_name:
        candidate = urdf_name if os.path.isabs(urdf_name) else os.path.join(urdf_dir, urdf_name)
        if not os.path.exists(candidate):
            raise RuntimeError(f"Requested URDF does not exist: {candidate}")
        return candidate

    preferred_candidates = []
    if robot_type == 'quad_mini_tuned':
        preferred_candidates.append(os.path.join(urdf_dir, 'robotSTL.urdf'))
    preferred_candidates.append(os.path.join(urdf_dir, 'robot.urdf'))

    for candidate in preferred_candidates:
        if os.path.exists(candidate):
            return candidate

    raise RuntimeError(f"No URDF found for robot_type={robot_type} under {urdf_dir}.")


def _resolve_task_file(user_command_share, robot_type, task_file_name):
    config_dir = os.path.join(user_command_share, 'config', robot_type)
    requested_name = task_file_name if task_file_name else 'task.info'
    candidate = requested_name if os.path.isabs(requested_name) else os.path.join(config_dir, requested_name)
    if not os.path.exists(candidate):
        raise RuntimeError(f"Requested task file does not exist: {candidate}")
    return candidate


def _extract_scalar_global(config_path, key):
    if not os.path.exists(config_path):
        return None
    pattern = re.compile(rf'^\s*{re.escape(key)}\s+([^\s;]+)', re.MULTILINE)
    match = pattern.search(open(config_path, 'r', encoding='utf-8').read())
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def _extract_scalar_from_block(config_path, block_name, key):
    if not os.path.exists(config_path):
        return None
    text = open(config_path, 'r', encoding='utf-8').read()
    block_match = re.search(
        rf'{re.escape(block_name)}\s*\{{(?P<body>.*?)\}}',
        text,
        re.DOTALL,
    )
    if not block_match:
        return None
    key_match = re.search(rf'^\s*{re.escape(key)}\s+([^\s;]+)', block_match.group('body'), re.MULTILINE)
    if not key_match:
        return None
    try:
        return float(key_match.group(1))
    except ValueError:
        return None


def _resolve_gazebo_base_gains(simulator_file, rl_file, control_type):
    default_kp = 10.0
    default_kd = 0.30

    if control_type == 'rl':
        rl_kp = _extract_scalar_global(rl_file, 'mujocoBaseKp')
        rl_kd = _extract_scalar_global(rl_file, 'mujocoBaseKd')
        if rl_kp is not None and rl_kd is not None:
            return rl_kp, rl_kd

    sim_kp = _extract_scalar_from_block(simulator_file, 'pid', 'kp')
    sim_kd = _extract_scalar_from_block(simulator_file, 'pid', 'kd')
    return sim_kp if sim_kp is not None else default_kp, sim_kd if sim_kd is not None else default_kd


def _create_nodes(context):
    robot_type_value = LaunchConfiguration('robot_type').perform(context)
    backend_value = LaunchConfiguration('backend').perform(context)
    control_type_value = LaunchConfiguration('control_type').perform(context).lower()
    terrain_value = LaunchConfiguration('mujoco_terrain').perform(context).lower()
    xml_override = LaunchConfiguration('mujoco_xml_file').perform(context)
    urdf_override = LaunchConfiguration('urdf_file').perform(context)
    task_file_name = LaunchConfiguration('task_file_name').perform(context)

    mujoco_share = get_package_share_directory('mujoco_simulator')
    user_command_share = get_package_share_directory('user_command')
    launch_share = get_package_share_directory('launch_simulation')

    simulator_file = os.path.join(user_command_share, 'config', robot_type_value, 'simulation.info')
    urdf_file = _resolve_urdf_file(mujoco_share, robot_type_value, urdf_override)
    task_file = _resolve_task_file(user_command_share, robot_type_value, task_file_name)
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
                    {'use_sim_time': LaunchConfiguration('gazebo_use_sim_time')},
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

    if backend_value == 'gazebo':
        odom_topic_value = LaunchConfiguration('odom_topic').perform(context) or 'odom'
        gazebo_base_kp, gazebo_base_kd = _resolve_gazebo_base_gains(
            simulator_file, rl_file, control_type_value
        )
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_share, 'launch', 'quad_mini_tuned_gz.launch.py')
            ),
            launch_arguments={
                'robot_type': robot_type_value,
                'robot_name': LaunchConfiguration('gazebo_robot_name'),
                'urdf_name': LaunchConfiguration('gazebo_urdf_name'),
                'headless': LaunchConfiguration('gazebo_headless'),
                'use_sim_time': LaunchConfiguration('gazebo_use_sim_time'),
                'world_name': LaunchConfiguration('gazebo_world_name'),
                'spawn_x': LaunchConfiguration('gazebo_spawn_x'),
                'spawn_y': LaunchConfiguration('gazebo_spawn_y'),
                'spawn_z': LaunchConfiguration('gazebo_spawn_z'),
                'spawn_yaw': LaunchConfiguration('gazebo_spawn_yaw'),
                'controller_base_kp': str(gazebo_base_kp),
                'controller_base_kd': str(gazebo_base_kd),
                'publish_robot_state': 'true',
            }.items(),
        )

        return [
            gazebo_launch,
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
                    {'jointFeedbackSource': 'joint_state'},
                    {'jointStateTopic': LaunchConfiguration('joint_state_topic')},
                    {'imuTopic': LaunchConfiguration('imu_topic')},
                    {'odomTopic': odom_topic_value},
                    {'hardwareCommandTopic': LaunchConfiguration('hardware_command_topic')},
                ]
            ),
        ]

    raise RuntimeError("backend must be 'sim', 'real', or 'gazebo'.")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='b1'),
        DeclareLaunchArgument('backend', default_value='sim'),
        DeclareLaunchArgument('contact_source', default_value='mujoco'),
        DeclareLaunchArgument('publish_rate_hz', default_value='0.0'),
        DeclareLaunchArgument('debug_state_logging', default_value='false'),
        DeclareLaunchArgument('control_type', default_value='mpc'),
        DeclareLaunchArgument('task_file_name', default_value='task.info'),
        DeclareLaunchArgument('urdf_file', default_value=''),
        DeclareLaunchArgument('mujoco_terrain', default_value='flat'),
        DeclareLaunchArgument('mujoco_xml_file', default_value=''),
        DeclareLaunchArgument('joint_feedback_source', default_value='joint_trajectory'),
        DeclareLaunchArgument('joint_feedback_topic', default_value='htdw_joint_cmd'),
        DeclareLaunchArgument('joint_state_topic', default_value='joint_states'),
        DeclareLaunchArgument('imu_topic', default_value='imu/data'),
        DeclareLaunchArgument('odom_topic', default_value=''),
        DeclareLaunchArgument('hardware_command_topic', default_value='bridge_joint_command'),
        DeclareLaunchArgument('gazebo_robot_name', default_value=''),
        DeclareLaunchArgument('gazebo_urdf_name', default_value=''),
        DeclareLaunchArgument('gazebo_headless', default_value='false'),
        DeclareLaunchArgument('gazebo_use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_world_name', default_value='empty'),
        DeclareLaunchArgument('gazebo_spawn_x', default_value='0.0'),
        DeclareLaunchArgument('gazebo_spawn_y', default_value='0.0'),
        DeclareLaunchArgument('gazebo_spawn_z', default_value='0.32'),
        DeclareLaunchArgument('gazebo_spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('mujoco_timestep', default_value='0.0'),
        DeclareLaunchArgument('mujoco_control_frequency', default_value='0.0'),
        DeclareLaunchArgument('mujoco_base_kp', default_value='0.0'),
        DeclareLaunchArgument('mujoco_base_kd', default_value='0.0'),
        OpaqueFunction(function=_create_nodes),
    ])
