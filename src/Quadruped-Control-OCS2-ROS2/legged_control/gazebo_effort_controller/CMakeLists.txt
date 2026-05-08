cmake_minimum_required(VERSION 3.8)
project(gazebo_effort_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(legged_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_control_effort_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  legged_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

pluginlib_export_plugin_description_file(controller_interface gazebo_effort_controller_plugins.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  hardware_interface
  legged_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

ament_package()
