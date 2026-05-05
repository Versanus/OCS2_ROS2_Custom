cmake_minimum_required(VERSION 3.8)
project(one_leg_pinocchio_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stm2ros REQUIRED)

add_executable(one_leg_inverse_dynamics_node
  src/one_leg_inverse_dynamics_node.cpp
)
target_compile_features(one_leg_inverse_dynamics_node PUBLIC cxx_std_17)
ament_target_dependencies(one_leg_inverse_dynamics_node
  rcl_interfaces
  rclcpp
  sensor_msgs
  stm2ros
)
target_link_libraries(one_leg_inverse_dynamics_node
  pinocchio::pinocchio
)

install(TARGETS one_leg_inverse_dynamics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/one_leg_control_gui.py
  scripts/one_leg_state_to_model_joint_state.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  go2_description-humble
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
