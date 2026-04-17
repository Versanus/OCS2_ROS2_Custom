#include "motion_control/controller/MpcBackend.h"

#include <stdexcept>

#include "motion_control/ros_interfaces/MPC_WBC_ROS_Interface.h"

MpcBackend::MpcBackend(const rclcpp::Node::SharedPtr& node) : node_(node) {}

MpcBackend::~MpcBackend() = default;

bool MpcBackend::configure(const ControllerConfig& config) {
  if (!node_) {
    return false;
  }

  if (config.taskFile.empty() || config.urdfFile.empty() || config.referenceFile.empty() || config.simulatorFile.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "MPC backend requires non-empty taskFile, urdfFile, referenceFile, and simulatorFile parameters.");
    return false;
  }

  config_ = config;
  mpcInterface_ = std::make_unique<MPC_WBC_ROS_Interface>(node_, config.taskFile, config.urdfFile, config.referenceFile,
                                                          config.simulatorFile, config.robotName);
  return true;
}

void MpcBackend::launch() {
  if (!mpcInterface_) {
    throw std::runtime_error("MPC backend was launched before configure() completed successfully.");
  }

  mpcInterface_->launchNodes();
}

const char* MpcBackend::name() const {
  return "mpc";
}
