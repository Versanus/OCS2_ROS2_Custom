#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mujoco_simulator/MujocoSimulation.hpp"

#include <cmath>
#include <exception>
#include <string>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace {
double positiveOverride(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

void loadRlRuntimeOptions(const std::string& rlFile,
                          const std::string& controlType,
                          const rclcpp::Logger& logger,
                          MujocoSimulation::RuntimeOptions& runtimeOptions) {
    if (controlType != "rl") {
        return;
    }

    runtimeOptions.directPositionControl = true;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_info(rlFile, pt);
    } catch (const std::exception& e) {
        RCLCPP_WARN(logger, "Could not read RL runtime settings from '%s': %s", rlFile.c_str(), e.what());
        return;
    }

    runtimeOptions.timestep = pt.get<double>("mujocoTimestep", runtimeOptions.timestep);
    runtimeOptions.controlFrequency = pt.get<double>("mujocoControlFrequency", runtimeOptions.controlFrequency);
    runtimeOptions.baseKp = pt.get<double>("mujocoBaseKp", runtimeOptions.baseKp);
    runtimeOptions.baseKd = pt.get<double>("mujocoBaseKd", runtimeOptions.baseKd);
    runtimeOptions.directPositionControl =
        pt.get<bool>("mujocoDirectPositionControl", runtimeOptions.directPositionControl);

    RCLCPP_INFO(logger,
                "Loaded RL MuJoCo settings from rl.info: timestep=%.6f control_frequency=%.2f base_kp=%.3f base_kd=%.3f direct_position_control=%s.",
                runtimeOptions.timestep, runtimeOptions.controlFrequency, runtimeOptions.baseKp, runtimeOptions.baseKd,
                runtimeOptions.directPositionControl ? "true" : "false");
}
}  // namespace

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    const std::string robotName = "legged_robot";
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        robotName + "_simulator",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    const std::string xmlFile = node->get_parameter("xmlFile").as_string();
    const std::string simulatorFile = node->get_parameter("simulatorFile").as_string();
    const std::string rlFile = node->get_parameter_or<std::string>("rlFile", "");
    const std::string controlType = node->get_parameter_or<std::string>("controlType", "mpc");
    MujocoSimulation::RuntimeOptions runtimeOptions;
    loadRlRuntimeOptions(rlFile, controlType, node->get_logger(), runtimeOptions);
    runtimeOptions.timestep = positiveOverride(node->get_parameter_or<double>("mujocoTimestep", 0.0), runtimeOptions.timestep);
    runtimeOptions.controlFrequency =
        positiveOverride(node->get_parameter_or<double>("mujocoControlFrequency", 0.0), runtimeOptions.controlFrequency);
    runtimeOptions.baseKp = positiveOverride(node->get_parameter_or<double>("mujocoBaseKp", 0.0), runtimeOptions.baseKp);
    runtimeOptions.baseKd = positiveOverride(node->get_parameter_or<double>("mujocoBaseKd", 0.0), runtimeOptions.baseKd);
    // const std::string xmlFile = "/home/zhx/Desktop/zhx_legged_ocs2_master/src/legged_control/mujoco_simulator/models/b1/urdf/robot.xml";
    // const std::string simulatorFile = "/home/zhx/Desktop/zhx_legged_ocs2_master/src/legged_control/user_command/config/b1/simulation.info";

    // Create MujocoSimulation instance
    MujocoSimulation mujoco_sim(node, xmlFile, simulatorFile, true, runtimeOptions);

    mujoco_sim.run();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
