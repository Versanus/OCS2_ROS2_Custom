cmake_minimum_required(VERSION 3.8)
project(new_robot)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(glfw3 REQUIRED)

get_filename_component(QUAD_OCS2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(MUJOCO_ROOT "${QUAD_OCS2_ROOT}/src/Quadruped-Control-OCS2-ROS2/mujoco/mujoco-3.2.2")
set(MUJOCO_INCLUDE_DIRS "${MUJOCO_ROOT}/include")
set(MUJOCO_LIBRARIES "${MUJOCO_ROOT}/lib/libmujoco.so")

if(NOT EXISTS "${MUJOCO_INCLUDE_DIRS}")
  message(FATAL_ERROR "MuJoCo headers not found: ${MUJOCO_INCLUDE_DIRS}")
endif()

if(NOT EXISTS "${MUJOCO_LIBRARIES}")
  message(FATAL_ERROR "MuJoCo library not found: ${MUJOCO_LIBRARIES}")
endif()

add_executable(passive_mujoco_viewer
  src/passive_mujoco_viewer.cpp
)
target_include_directories(passive_mujoco_viewer PRIVATE ${MUJOCO_INCLUDE_DIRS})
ament_target_dependencies(passive_mujoco_viewer ament_index_cpp)
target_link_libraries(passive_mujoco_viewer ${MUJOCO_LIBRARIES} glfw)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    robot.urdf
    robot_raw.urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS passive_mujoco_viewer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
