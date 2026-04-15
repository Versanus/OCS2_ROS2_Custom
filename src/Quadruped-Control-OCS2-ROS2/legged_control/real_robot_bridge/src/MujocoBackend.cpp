#include "real_robot_bridge/MujocoBackend.h"

#include <algorithm>
#include <stdexcept>

namespace real_robot_bridge {
namespace {

constexpr double kRealtimeToleranceSec = 1e-6;

}  // namespace

MujocoBackend::MujocoBackend(const std::string& xml_file, const std::string& simulator_file)
    : sim_node_(rclcpp::Node::make_shared(
          "mujoco_bridge_backend_internal",
          rclcpp::NodeOptions().use_global_arguments(false))),
      last_render_time_(std::chrono::steady_clock::now()) {
  if (xml_file.empty()) {
    throw std::runtime_error("MujocoBackend requires a non-empty xmlFile.");
  }
  if (simulator_file.empty()) {
    throw std::runtime_error("MujocoBackend requires a non-empty simulatorFile.");
  }

  simulation_ = std::make_unique<MujocoSimulation>(sim_node_, xml_file, simulator_file, false);
  control_period_sec_ = 1.0 / std::max(1.0, simulation_->getControlFrequency());
  render_period_sec_ = 1.0 / std::max(1.0, simulation_->getRenderFrequency());
  simulation_->stepControlPeriod();
  simulation_->render();
  sim_time_reference_ = simulation_->getData()->time;
  wall_time_reference_ = std::chrono::steady_clock::now();
}

double MujocoBackend::defaultPublishRateHz() const {
  return std::max(1.0, simulation_->getControlFrequency());
}

void MujocoBackend::prepareForControlStart() {
  *simulation_->getSimuState() = false;
  *simulation_->getContrlState() = false;
  sim_time_reference_ = simulation_->getData()->time;
  wall_time_reference_ = std::chrono::steady_clock::now();
}

void MujocoBackend::writeCommand(const legged_msgs::msg::JointControlData& command) {
  simulation_->applyJointControl(command);
}

void MujocoBackend::update() {
  if (*simulation_->getContrlState() || *simulation_->getSimuState()) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_wall_time =
        std::chrono::duration<double>(now - wall_time_reference_).count();
    const double allowed_sim_time = sim_time_reference_ + elapsed_wall_time;

    while (simulation_->getData()->time + control_period_sec_ <= allowed_sim_time + kRealtimeToleranceSec) {
      simulation_->stepControlPeriod();
    }
  }

  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_render_time_).count() >= render_period_sec_) {
    simulation_->render();
    last_render_time_ = now;
  }
}

bool MujocoBackend::read(BackendData& data) {
  simulation_->populate_state_message(data.state);
  simulation_->populate_sensor_message(data.sensor);
  return true;
}

}  // namespace real_robot_bridge
