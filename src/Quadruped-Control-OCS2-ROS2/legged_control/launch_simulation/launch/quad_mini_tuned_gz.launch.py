import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GAZEBO_COMMAND_JOINT_NAMES = [
    "LF_HAA", "LF_HFE", "LF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
]


def _load_robot_description(urdf_file, mesh_dir):
    with open(urdf_file, "r", encoding="utf-8") as file_handle:
        robot_description = file_handle.read()
    robot_description = robot_description.replace('filename="meshes/', f'filename="{mesh_dir}/')
    robot_description = robot_description.replace('filename="package://', 'filename="package://')
    return robot_description


def _resolve_urdf_file(urdf_dir, robot_type, requested_urdf_name):
    if requested_urdf_name:
        candidate = requested_urdf_name
        if not os.path.isabs(candidate):
            candidate = os.path.join(urdf_dir, candidate)
        if not os.path.exists(candidate):
            raise RuntimeError(f"Requested URDF does not exist: {candidate}")
        return candidate

    preferred_candidates = []
    if robot_type == "quad_mini_tuned":
        preferred_candidates.append(os.path.join(urdf_dir, "robot_gz.urdf"))
        preferred_candidates.append(os.path.join(urdf_dir, "robotSTL_gz.urdf"))
        preferred_candidates.append(os.path.join(urdf_dir, "robotSTL.urdf"))
    preferred_candidates.append(os.path.join(urdf_dir, "robot.urdf"))

    for candidate in preferred_candidates:
        if os.path.exists(candidate):
            return candidate

    raise RuntimeError(f"No Gazebo URDF found under {urdf_dir}.")


def _create_launch_items(context):
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_type = LaunchConfiguration("robot_type").perform(context)
    requested_robot_name = LaunchConfiguration("robot_name").perform(context)
    requested_urdf_name = LaunchConfiguration("urdf_name").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    world_name = LaunchConfiguration("world_name").perform(context)
    robot_name = requested_robot_name if requested_robot_name else robot_type

    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    mujoco_share = get_package_share_directory("mujoco_simulator")

    model_dir = os.path.join(mujoco_share, "models", robot_type)
    urdf_dir = os.path.join(model_dir, "urdf")
    mesh_dir = os.path.join(model_dir, "meshes")
    urdf_file = _resolve_urdf_file(urdf_dir, robot_type, requested_urdf_name)

    robot_description = _load_robot_description(urdf_file, mesh_dir)
    gz_args = "-r -s empty.sdf" if headless else "-r empty.sdf"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(LaunchConfiguration("publish_robot_state")),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"{robot_name}_spawn",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
        arguments=[
            "-name", robot_name,
            "-world", world_name,
            "-param", "robot_description",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"{robot_name}_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/joint_states_gz@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ] + [
            f"/model/{robot_name}/joint/{joint_name}/cmd_force@std_msgs/msg/Float64]gz.msgs.Double"
            for joint_name in GAZEBO_COMMAND_JOINT_NAMES
        ],
        remappings=[
            ("/joint_states_gz", "joint_states_raw"),
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("joint_gui")),
    )

    return [
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.pathsep.join([model_dir, urdf_dir, mesh_dir]),
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            os.pathsep.join([model_dir, urdf_dir, mesh_dir]),
        ),
        LogInfo(msg=f"Gazebo launch robot_type={robot_type} robot_name={robot_name} urdf_file={urdf_file}"),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        joint_state_publisher_gui,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="quad_mini_tuned"),
            DeclareLaunchArgument("robot_name", default_value=""),
            DeclareLaunchArgument("urdf_name", default_value=""),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("joint_gui", default_value="false"),
            DeclareLaunchArgument("publish_robot_state", default_value="true"),
            DeclareLaunchArgument("world_name", default_value="empty"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.32"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            OpaqueFunction(function=_create_launch_items),
        ]
    )
