import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

package_description = "mujoco_simulator"

def process_xacro():
    pkg_path = os.path.join(get_package_share_directory(package_description))
    xacro_file = os.path.join(pkg_path, 'models', 'quad_mini', 'urdf', 'robot.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    return robot_description_config.toxml()

def generate_launch_description():

    print("\n" + "="*50)
    print("Lütfen çalıştırmak istediğiniz modu seçin:")
    print("1 - real2sim (Real to Simulation)")
    print("2 - sim2real (Simulation to Real)")
    print("3 - sim2real (Simulation to Real) with controller")
    print("="*50)

    mode = input("Seçiminiz (1, 2 veya 3): ").strip()

    rviz_config_file = os.path.join(
        get_package_share_directory(package_description),
        "config",
        "visualize_urdf.rviz"
    )

    robot_description = process_xacro()

    nodes = []

    # ============================================================
    #                           MODE 1
    # ============================================================
    if mode == "1":
        print("real2sim modu başlatılıyor...")

        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz_ocs2',
                output='screen',
                arguments=["-d", rviz_config_file]
            )
        )

        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        'publish_frequency': 100.0,
                        'use_tf_static': True,
                        'robot_description': robot_description
                    }
                ],
            )
        )

        # nodes.append(
        #     Node(
        #         package='hardware_interface',
        #         executable='digitalTwin.py',
        #         name='digital_twin_bridge',
        #         output='screen',
        #         parameters=[{'mode': 'real2sim'}]
        #     )
        # )

    # ============================================================
    #                           MODE 2
    # ============================================================
    elif mode == "2":
        print("sim2real modu başlatılıyor...")

        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz_ocs2',
                output='screen',
                arguments=["-d", rviz_config_file]
            )
        )

        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        'publish_frequency': 100.0,
                        'use_tf_static': True,
                        'robot_description': robot_description
                    }
                ],
            )
        )

        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher',
                output='screen',
            )
        )

        nodes.append(
            Node(
                package='hardware_interface',
                executable='digitalTwin.py',
                name='digital_twin_bridge',
                output='screen',
                parameters=[{'mode': 'sim2real'}]
            )
        )
        nodes.append(
            Node(
                package='hardware_interface',
                executable='motor_status.py',
                name='motor_monitor',
                output='screen',
            )
        )
        nodes.append(
            Node(
                package='hardware_interface',
                executable='joint_status.py',
                name='joint_state_comparator',
                output='screen',
            )
        )

        nodes.append(
            Node(
                package='hardware_interface',
                executable='gui_status.py',
                name='motor_status_gui',
                output='screen',
            )
        )

    # ============================================================
    #                           MODE 3
    # ============================================================
    elif mode == "3":
        print("sim2real with controller modu başlatılıyor...")
        print("Bu modda sadece controller.launch.py çalıştırılacaktır.")

        controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("hardware_interface"),
                    "launch",
                    "controller.launch.py"
                )
            )
        )

        # Teleop.py yeni terminalde
        teleop_terminal = ExecuteProcess(
            cmd=[
                "gnome-terminal",
                "--",
                "bash",
                "-c",
                "cd ~/digital_twin_ws/src/champ_teleop && python3 teleop.py; exec bash"
            ],
            output='screen'
        )

        digital_twin_node =Node(
                package='hardware_interface',
                executable='digitalTwin.py',
                name='digital_twin_bridge',
                output='screen',
                parameters=[{'mode': 'controller'}]
            )
        

        # motor_status_gui node'u
        motor_status_node = Node(
            package='hardware_interface',
            executable='gui_status.py',
            name='motor_status_gui',
            output='screen',
        )

        nodes = [
            controller_launch,
            teleop_terminal,
            digital_twin_node,
            motor_status_node
        ]


    else:
        print(f"Geçersiz seçim: {mode}, varsayılan olarak real2sim çalıştırılıyor...")

    return LaunchDescription(nodes)
