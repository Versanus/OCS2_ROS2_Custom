#pragma once

#include <memory>
#include <string>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>

#include "motion_control/legged_estimation/EstimatorConfig.h"

class StateEstimateBase;

class StateEstimatorFactory {
 public:
  static std::shared_ptr<StateEstimateBase> create(const EstimatorConfig& estimatorConfig,
                                                   const rclcpp::Node::SharedPtr& node,
                                                   ocs2::PinocchioInterface pinocchioInterface,
                                                   ocs2::CentroidalModelInfo info,
                                                   const ocs2::PinocchioEndEffectorKinematics& eeKinematics,
                                                   const std::string& configFile,
                                                   bool verbose);
};
