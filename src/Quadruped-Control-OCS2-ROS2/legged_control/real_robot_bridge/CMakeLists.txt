cmake_minimum_required(VERSION 3.8)
project(real_robot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(legged_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(mujoco_simulator REQUIRED)

add_library(real_robot_bridge_common SHARED
  src/BridgeNodeBase.cpp
  src/ContactEstimator.cpp
  src/HardwareBackend.cpp
)
target_include_directories(real_robot_bridge_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
  real_robot_bridge_common
  builtin_interfaces
  rclcpp
  sensor_msgs
  trajectory_msgs
  nav_msgs
  std_msgs
  legged_msgs
  ocs2_core
)

add_library(real_robot_bridge_sim_backend SHARED
  src/MujocoBackend.cpp
)
target_include_directories(real_robot_bridge_sim_backend
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
  real_robot_bridge_sim_backend
  rclcpp
  legged_msgs
)
target_link_libraries(real_robot_bridge_sim_backend mujoco_simulator::mujoco_simulator_core)

add_executable(bridge_sim_node src/bridge_sim_main.cpp)
ament_target_dependencies(bridge_sim_node builtin_interfaces rclcpp)
target_link_libraries(bridge_sim_node real_robot_bridge_common real_robot_bridge_sim_backend)

add_executable(bridge_real_node src/bridge_real_main.cpp)
ament_target_dependencies(bridge_real_node builtin_interfaces rclcpp)
target_link_libraries(bridge_real_node real_robot_bridge_common)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  real_robot_bridge_common
  real_robot_bridge_sim_backend
  bridge_sim_node
  bridge_real_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  rclcpp
  sensor_msgs
  trajectory_msgs
  nav_msgs
  std_msgs
  legged_msgs
  ocs2_core
  Boost
  Eigen3
  mujoco_simulator
)

ament_package()
