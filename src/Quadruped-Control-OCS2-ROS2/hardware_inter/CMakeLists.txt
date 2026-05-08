cmake_minimum_required(VERSION 3.8)
project(hardware_inter)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/gui_status.py
  scripts/gazebo_joint_effort_adapter.py
  scripts/joint_status.py
  scripts/joint_plotter.py
  scripts/motor_status.py
  scripts/digitalTwin.py
  scripts/mujoco_to_joint_state.py
  scripts/mpc_digital_twin.py
  scripts/odom_to_tf.py
  scripts/odom_to_path.py
  scripts/odom_relay.py
  scripts/sensor_to_joint_state.py
  scripts/state_to_odom.py
  scripts/sim2real.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
