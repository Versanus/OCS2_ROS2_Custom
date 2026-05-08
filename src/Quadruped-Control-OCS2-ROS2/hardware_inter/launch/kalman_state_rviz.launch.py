import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


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

    raise RuntimeError(f"No RViz URDF found for robot_type={robot_type} under {urdf_dir}.")


def _create_nodes(context):
    mujoco_share = get_package_share_directory('mujoco_simulator')
    hardware_share = get_package_share_directory('hardware_inter')

    robot_type = LaunchConfiguration('robot_type').perform(context)
    urdf_name = LaunchConfiguration('urdf_name').perform(context)
    joint_source = LaunchConfiguration('joint_source')
    odom_source = LaunchConfiguration('odom_source')
    sensor_input_topic = LaunchConfiguration('sensor_input_topic')
    state_input_topic = LaunchConfiguration('state_input_topic')
    input_joint_state_topic = LaunchConfiguration('input_joint_state_topic')
    output_joint_state_topic = LaunchConfiguration('output_joint_state_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    path_topic = LaunchConfiguration('path_topic')

    urdf_file = _resolve_urdf_file(mujoco_share, robot_type, urdf_name)
    rviz_config_file = os.path.join(hardware_share, 'rviz', 'kalman_state.rviz')
    mesh_dir = os.path.join(mujoco_share, 'models', robot_type, 'meshes')

    with open(urdf_file, 'r', encoding='utf-8') as urdf_handle:
        robot_description = urdf_handle.read()
    robot_description = robot_description.replace(
        'filename="meshes/',
        f'filename="file://{mesh_dir}/'
    )

    return [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_ocs2',
            output='screen',
            arguments=['-d', rviz_config_file],
            remappings=[
                ('/odom', odom_topic),
                ('/odom_path', path_topic),
            ],
        ),
        Node(
            package='hardware_inter',
            executable='sensor_to_joint_state.py',
            name='sensor_to_joint_state',
            output='screen',
            condition=IfCondition(PythonExpression(["'", joint_source, "' == 'sensor'"])),
            parameters=[
                {'input_topic': sensor_input_topic},
                {'output_topic': output_joint_state_topic},
                {'publish_velocity': True},
            ],
        ),
        Node(
            package='hardware_inter',
            executable='mujoco_to_joint_state.py',
            name='state_to_joint_state',
            output='screen',
            condition=IfCondition(PythonExpression(["'", joint_source, "' == 'state'"])),
            parameters=[
                {'input_topic': state_input_topic},
                {'output_topic': output_joint_state_topic},
                {'publish_velocity': True},
                {'publish_effort': False},
            ],
        ),
        Node(
            package='hardware_inter',
            executable='digitalTwin.py',
            name='digital_twin_bridge',
            output='screen',
            condition=IfCondition(PythonExpression(["'", joint_source, "' == 'hardware'"])),
            parameters=[
                {'mode': 'real2sim'},
                {'input_joint_state_topic': input_joint_state_topic},
                {'output_joint_state_topic': output_joint_state_topic},
                {'output_legged_joint_names': True},
            ],
        ),
        Node(
            package='hardware_inter',
            executable='state_to_odom.py',
            name='state_to_odom_bridge',
            output='screen',
            condition=IfCondition(PythonExpression(["'", odom_source, "' == 'state'"])),
            parameters=[
                {'input_topic': state_input_topic},
                {'output_topic': odom_topic},
            ],
        ),
        Node(
            package='hardware_inter',
            executable='odom_to_tf.py',
            name='odom_to_tf_bridge',
            output='screen',
            parameters=[
                {'odom_topic': odom_topic},
            ],
        ),
        Node(
            package='hardware_inter',
            executable='odom_to_path.py',
            name='odom_to_path_bridge',
            output='screen',
            parameters=[
                {'odom_topic': odom_topic},
                {'path_topic': path_topic},
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            remappings=[
                ('joint_states', output_joint_state_topic),
            ],
            parameters=[
                {
                    'publish_frequency': 100.0,
                    'use_tf_static': True,
                    'robot_description': robot_description,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value='quad_mini_real'),
        DeclareLaunchArgument('urdf_name', default_value=''),
        DeclareLaunchArgument('joint_source', default_value='sensor'),
        DeclareLaunchArgument('odom_source', default_value='topic'),
        DeclareLaunchArgument('sensor_input_topic', default_value='simulator_sensor_data'),
        DeclareLaunchArgument('state_input_topic', default_value='simulator_state_data'),
        DeclareLaunchArgument('input_joint_state_topic', default_value='htdw_joint_state'),
        DeclareLaunchArgument('output_joint_state_topic', default_value='joint_states'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('path_topic', default_value='odom_path'),
        OpaqueFunction(function=_create_nodes),
    ])
