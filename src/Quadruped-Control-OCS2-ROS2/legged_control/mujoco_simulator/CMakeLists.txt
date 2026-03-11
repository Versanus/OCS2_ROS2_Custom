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

add_executable(mujoco_simulator src/SimulatorNode.cpp src/MujocoSimulation.cpp)
ament_target_dependencies(mujoco_simulator rclcpp std_msgs legged_msgs ocs2_core Boost) #ros2 link library
target_link_libraries(mujoco_simulator ${MUJOCO_LIBRARIES} glfw) #cmake link library

install(TARGETS mujoco_simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models DESTINATION share/${PROJECT_NAME}/)


ament_package()
