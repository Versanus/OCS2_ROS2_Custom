#include "real_robot_bridge/BridgeNodeBase.h"
#include "real_robot_bridge/MujocoBackend.h"

#include <cmath>
#include <exception>
#include <string>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace real_robot_bridge {

namespace {
double positiveOverride(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

void loadRlRuntimeOptions(const std::string& rl_file,
                          const std::string& control_type,
                          const rclcpp::Logger& logger,
                          MujocoSimulation::RuntimeOptions& runtime_options) {
  if (control_type != "rl") {
    return;
  }

  runtime_options.directPositionControl = true;

  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_info(rl_file, pt);
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger, "Could not read RL runtime settings from '%s': %s", rl_file.c_str(), e.what());
    return;
  }

  runtime_options.timestep = pt.get<double>("mujocoTimestep", runtime_options.timestep);
  runtime_options.controlFrequency = pt.get<double>("mujocoControlFrequency", runtime_options.controlFrequency);
  runtime_options.baseKp = pt.get<double>("mujocoBaseKp", runtime_options.baseKp);
  runtime_options.baseKd = pt.get<double>("mujocoBaseKd", runtime_options.baseKd);
  runtime_options.directPositionControl =
      pt.get<bool>("mujocoDirectPositionControl", runtime_options.directPositionControl);

  RCLCPP_INFO(logger,
              "Loaded RL MuJoCo settings from rl.info: timestep=%.6f control_frequency=%.2f base_kp=%.3f base_kd=%.3f direct_position_control=%s.",
              runtime_options.timestep, runtime_options.controlFrequency, runtime_options.baseKp, runtime_options.baseKd,
              runtime_options.directPositionControl ? "true" : "false");
}
}  // namespace

class SimBridgeNode final : public BridgeNodeBase {
 public:
  explicit SimBridgeNode(const rclcpp::NodeOptions& options)
      : BridgeNodeBase("bridge_sim_node", options) {
    const auto xml_file = declare_parameter<std::string>("xmlFile", "");
    const auto simulator_file = declare_parameter<std::string>("simulatorFile", "");
    const auto rl_file = declare_parameter<std::string>("rlFile", "");
    const auto control_type = declare_parameter<std::string>("controlType", "mpc");
    MujocoSimulation::RuntimeOptions runtime_options;
    loadRlRuntimeOptions(rl_file, control_type, get_logger(), runtime_options);
    runtime_options.timestep = positiveOverride(declare_parameter<double>("mujocoTimestep", 0.0), runtime_options.timestep);
    runtime_options.controlFrequency =
        positiveOverride(declare_parameter<double>("mujocoControlFrequency", 0.0), runtime_options.controlFrequency);
    runtime_options.baseKp = positiveOverride(declare_parameter<double>("mujocoBaseKp", 0.0), runtime_options.baseKp);
    runtime_options.baseKd = positiveOverride(declare_parameter<double>("mujocoBaseKd", 0.0), runtime_options.baseKd);
    RCLCPP_INFO(get_logger(), "Using MuJoCo XML: %s", xml_file.c_str());
    initializeBackend(std::make_unique<MujocoBackend>(xml_file, simulator_file, runtime_options));
  }
};

}  // namespace real_robot_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<real_robot_bridge::SimBridgeNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
