import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


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


def _create_nodes(context):
    robot_type_value = LaunchConfiguration('robot_type').perform(context)
    control_type_value = LaunchConfiguration('control_type').perform(context).lower()
    terrain_value = LaunchConfiguration('mujoco_terrain').perform(context).lower()
    xml_override = LaunchConfiguration('mujoco_xml_file').perform(context)

    mujoco_share = get_package_share_directory('mujoco_simulator')
    user_command_share = get_package_share_directory('user_command')

    xml_file = _resolve_xml_file(mujoco_share, robot_type_value, control_type_value, terrain_value, xml_override)
    simulator_file = os.path.join(user_command_share, 'config', robot_type_value, 'simulation.info')
    rl_file = os.path.join(user_command_share, 'config', robot_type_value, 'rl.info')

    return [
        Node(
            package='mujoco_simulator',
            executable='mujoco_simulator',
            name='mujoco_simulator',
            output='screen',
            parameters=[
                {'xmlFile': xml_file},
                {'simulatorFile': simulator_file},
                {'rlFile': rl_file},
                {'controlType': LaunchConfiguration('control_type')},
                {'mujocoTimestep': ParameterValue(LaunchConfiguration('mujoco_timestep'), value_type=float)},
                {'mujocoControlFrequency': ParameterValue(LaunchConfiguration('mujoco_control_frequency'), value_type=float)},
                {'mujocoBaseKp': ParameterValue(LaunchConfiguration('mujoco_base_kp'), value_type=float)},
                {'mujocoBaseKd': ParameterValue(LaunchConfiguration('mujoco_base_kd'), value_type=float)},
            ]
        )
    ]


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_type',
            default_value='quad_mini_tuned'
        ),
        DeclareLaunchArgument('control_type', default_value='mpc'),
        DeclareLaunchArgument('mujoco_terrain', default_value='flat'),
        DeclareLaunchArgument('mujoco_xml_file', default_value=''),
        DeclareLaunchArgument('mujoco_timestep', default_value='0.0'),
        DeclareLaunchArgument('mujoco_control_frequency', default_value='0.0'),
        DeclareLaunchArgument('mujoco_base_kp', default_value='0.0'),
        DeclareLaunchArgument('mujoco_base_kd', default_value='0.0'),
        OpaqueFunction(function=_create_nodes),
    ])
