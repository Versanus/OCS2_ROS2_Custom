cmake_minimum_required(VERSION 3.8)
project(hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# find_package(<dependency> REQUIRED)

add_executable(my_hardware src/my_hardware.cpp)
target_include_directories(my_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_hardware PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(my_hardware std_msgs)

install(TARGETS my_hardware
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(PROGRAMS
  scripts/gui_status.py
  scripts/joint_status.py
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
  scripts/read_to_csv.py
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
