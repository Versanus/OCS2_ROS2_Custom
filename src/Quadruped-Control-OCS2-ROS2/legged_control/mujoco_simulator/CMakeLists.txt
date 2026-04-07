cmake_minimum_required(VERSION 3.8)
project(mujoco_simulator)

get_filename_component(QUAD_OCS2_VENDOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(MUJOCO_ROOT "${QUAD_OCS2_VENDOR_ROOT}/mujoco/mujoco-3.2.2" CACHE PATH "MuJoCo root directory")
set(MUJOCO_INCLUDE_DIRS "${MUJOCO_ROOT}/include")
set(MUJOCO_LIBRARIES "${MUJOCO_ROOT}/lib/libmujoco.so")

if(NOT EXISTS "${MUJOCO_INCLUDE_DIRS}")
  message(FATAL_ERROR "MuJoCo headers not found: ${MUJOCO_INCLUDE_DIRS}")
endif()

if(NOT EXISTS "${MUJOCO_LIBRARIES}")
  message(FATAL_ERROR "MuJoCo library not found: ${MUJOCO_LIBRARIES}")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glfw3 REQUIRED)
find_package(legged_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
  include
  ${MUJOCO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mujoco_simulator_core SHARED
  src/MujocoSimulation.cpp
)
target_include_directories(mujoco_simulator_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(mujoco_simulator_core rclcpp std_msgs legged_msgs ocs2_core Boost)
target_link_libraries(mujoco_simulator_core ${MUJOCO_LIBRARIES} glfw)

add_executable(mujoco_simulator src/SimulatorNode.cpp)
ament_target_dependencies(mujoco_simulator rclcpp std_msgs legged_msgs ocs2_core Boost)
target_link_libraries(mujoco_simulator mujoco_simulator_core)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS mujoco_simulator_core mujoco_simulator
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models config DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs legged_msgs ocs2_core Boost Eigen3)

ament_package()
