cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(motion_control)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  legged_msgs
  #ocs2_msgs
  #ocs2_ros_interfaces
  #ocs2_legged_robot
  Boost
  angles
  nav_msgs
  visualization_msgs
)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
#find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(legged_msgs REQUIRED)
#find_package(ocs2_msgs REQUIRED)
#find_package(ocs2_legged_robot REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(inekf REQUIRED)



find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

get_filename_component(QUAD_OCS2_VENDOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(QUAD_OCS2_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../" ABSOLUTE)
set(QPOASES_ROOT "${QUAD_OCS2_VENDOR_ROOT}/qpOASES-master" CACHE PATH "qpOASES root directory")
set(QPOASES_INCLUDE_DIR "${QPOASES_ROOT}/include")
set(QPOASES_LIBRARY "${QPOASES_ROOT}/build/libs/libqpOASES.a")
set(ONNXRUNTIME_ROOT_DIR "${QUAD_OCS2_REPO_ROOT}/third_party/onnxruntime")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(ONNXRUNTIME_VARIANT "orin_cpu")
else()
  set(ONNXRUNTIME_VARIANT "pc_cpu")
endif()

set(ONNXRUNTIME_ROOT "${ONNXRUNTIME_ROOT_DIR}/${ONNXRUNTIME_VARIANT}")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
file(GLOB ONNXRUNTIME_LIBRARIES "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so*")
list(LENGTH ONNXRUNTIME_LIBRARIES ONNXRUNTIME_LIBRARY_COUNT)

if(NOT EXISTS "${QPOASES_INCLUDE_DIR}")
  message(FATAL_ERROR "qpOASES headers not found: ${QPOASES_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${QPOASES_LIBRARY}")
  message(FATAL_ERROR "qpOASES static library not found: ${QPOASES_LIBRARY}")
endif()

if(NOT EXISTS "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h")
  message(FATAL_ERROR "ONNX Runtime headers not found: ${ONNXRUNTIME_INCLUDE_DIR}")
endif()

if(ONNXRUNTIME_LIBRARY_COUNT EQUAL 0)
  message(FATAL_ERROR "ONNX Runtime shared library not found in ${ONNXRUNTIME_ROOT}/lib")
endif()

list(GET ONNXRUNTIME_LIBRARIES 0 ONNXRUNTIME_LIBRARY)
add_library(onnxruntime_local SHARED IMPORTED GLOBAL)
set_target_properties(onnxruntime_local PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)

include_directories(${QPOASES_INCLUDE_DIR})

# Add directories for all targets
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

###########
## Build ##
###########

# Main library
add_library(${PROJECT_NAME}
  src/common/ModelSettings.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/gait/GaitSchedule.cpp
  src/gait/Gait.cpp
  src/legged_interface/LeggedRobotInterface.cpp
  src/reference_manager/SwitchedModelReferenceManager.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SplineCpg.cpp
  src/initialization/LeggedRobotInitializer.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/robot_precomputation/LeggedRobotPreComputation.cpp
  src/dynamics/LeggedRobotDynamicsAD.cpp
  src/controller/LeggedControllerRosInterface.cpp
  src/controller/MpcBackend.cpp
  src/controller/OrtCpuRunner.cpp
  src/controller/RlBackend.cpp
  src/ros_interfaces/GaitReceiver.cpp
  src/ros_interfaces/RosReferenceManager.cpp
  src/ros_interfaces/MPC_WBC_ROS_Interface.cpp
  src/legged_wbc/WbcBase.cpp
  src/legged_wbc/WeightedWbc.cpp
  src/legged_wbc/HoQp.cpp
  src/legged_wbc/HierarchicalWbc.cpp
  src/legged_estimation/EstimatorConfig.cpp
  src/legged_estimation/StateEstimatorFactory.cpp
  src/legged_estimation/StateEstimateBase.cpp
  src/legged_estimation/LinearKalmanFilter.cpp
  src/legged_estimation/InvariantEkfEstimate.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_include_directories(${PROJECT_NAME} PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
# Keep qpOASES on the motion_control target as before.
target_link_libraries(${PROJECT_NAME}
  ${QPOASES_LIBRARY}
  inekf::inekf
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

## SQP-MPC node for legged robot
add_executable(legged_robot_sqp_mpc
  src/LeggedRobotSqpMpcNode.cpp
)

add_executable(legged_robot_controller
  src/LeggedRobotControllerNode.cpp
)

ament_target_dependencies(legged_robot_sqp_mpc
  ${dependencies}
)

ament_target_dependencies(legged_robot_controller
  ${dependencies}
)

# Link the main library to the node
target_link_libraries(legged_robot_sqp_mpc
  ${PROJECT_NAME}
  onnxruntime_local
)

target_link_libraries(legged_robot_controller
  ${PROJECT_NAME}
  onnxruntime_local
)

set_target_properties(legged_robot_sqp_mpc PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

set_target_properties(legged_robot_controller PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
)

target_compile_options(legged_robot_sqp_mpc PRIVATE ${OCS2_CXX_FLAGS})
target_compile_options(legged_robot_controller PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_legged_robot_ros")
   add_clang_tooling(
     TARGETS
       ${PROJECT_NAME}
       legged_robot_sqp_mpc
       legged_robot_controller
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS
    legged_robot_sqp_mpc
    legged_robot_controller
    DESTINATION lib/${PROJECT_NAME}
)
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY launch rviz
  #DESTINATION share/${PROJECT_NAME}/
#)

ament_export_dependencies(${dependencies})
ament_export_dependencies(inekf)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
