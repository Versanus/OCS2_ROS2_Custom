#include "motion_control/legged_estimation/StateEstimatorFactory.h"

#include <stdexcept>

#include "motion_control/legged_estimation/InvariantEkfEstimate.h"
#include "motion_control/legged_estimation/LinearKalmanFilter.h"

std::shared_ptr<StateEstimateBase> StateEstimatorFactory::create(
    const EstimatorConfig& estimatorConfig, const rclcpp::Node::SharedPtr& node,
    ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
    const ocs2::PinocchioEndEffectorKinematics& eeKinematics, const std::string& configFile, bool verbose) {
  switch (estimatorConfig.type) {
    case EstimatorType::Linear: {
      auto estimator = std::make_shared<KalmanFilterEstimate>(node, std::move(pinocchioInterface), std::move(info), eeKinematics);
      estimator->loadSettings(configFile, verbose);
      return estimator;
    }
    case EstimatorType::InEKF: {
      auto estimator = std::make_shared<InvariantEkfEstimate>(node, std::move(pinocchioInterface), std::move(info), eeKinematics,
                                                              estimatorConfig);
      estimator->loadSettings(configFile, verbose);
      return estimator;
    }
    default:
      throw std::runtime_error("Unsupported estimator type.");
  }
}
