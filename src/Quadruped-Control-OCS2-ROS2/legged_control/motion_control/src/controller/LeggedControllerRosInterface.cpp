#include "motion_control/controller/LeggedControllerRosInterface.h"

#include <stdexcept>
#include <utility>

#include "motion_control/controller/MpcBackend.h"
#include "motion_control/controller/RlBackend.h"

namespace {

std::string getRequiredStringParameter(const rclcpp::Node::SharedPtr& node, const char* parameter_name) {
  if (!node->has_parameter(parameter_name)) {
    throw std::runtime_error(std::string("Missing required parameter '") + parameter_name + "'.");
  }

  const auto parameter = node->get_parameter(parameter_name);
  const auto value = parameter.as_string();
  if (value.empty()) {
    throw std::runtime_error(std::string("Parameter '") + parameter_name + "' must not be empty.");
  }

  return value;
}

std::string getOptionalStringParameter(const rclcpp::Node::SharedPtr& node, const char* parameter_name,
                                       std::string default_value) {
  if (!node->has_parameter(parameter_name)) {
    return default_value;
  }

  const auto value = node->get_parameter(parameter_name).as_string();
  return value.empty() ? std::move(default_value) : value;
}

}  // namespace

LeggedControllerRosInterface::LeggedControllerRosInterface(const rclcpp::Node::SharedPtr& node) : node_(node) {}

void LeggedControllerRosInterface::launch() {
  config_ = loadConfig();
  backend_ = createBackend(config_);
  if (!backend_) {
    throw std::runtime_error("Failed to create a controller backend.");
  }

  if (!backend_->configure(config_)) {
    throw std::runtime_error(std::string("Failed to configure controller backend '") + backend_->name() + "'.");
  }

  RCLCPP_INFO(node_->get_logger(), "Launching controller backend '%s'.", backend_->name());
  backend_->launch();
}

ControllerConfig LeggedControllerRosInterface::loadConfig() const {
  ControllerConfig config;
  config.robotName = getOptionalStringParameter(node_, "robotName", config.robotName);
  config.controlType = getOptionalStringParameter(node_, "controlType", "mpc");
  config.taskFile = getRequiredStringParameter(node_, "taskFile");
  config.urdfFile = getRequiredStringParameter(node_, "urdfFile");
  config.referenceFile = getRequiredStringParameter(node_, "referenceFile");
  config.simulatorFile = getRequiredStringParameter(node_, "simulatorFile");
  config.rlConfigFile = getOptionalStringParameter(node_, "rlConfigFile", "");
  return config;
}

std::unique_ptr<ControllerBackend> LeggedControllerRosInterface::createBackend(const ControllerConfig& config) const {
  if (config.controlType == "mpc") {
    return std::make_unique<MpcBackend>(node_);
  }
  if (config.controlType == "rl") {
    return std::make_unique<RlBackend>(node_);
  }

  throw std::runtime_error("Unsupported controlType '" + config.controlType + "'.");
}
