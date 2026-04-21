#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mujoco_simulator/MujocoSimulation.hpp"
#include "real_robot_bridge/BackendBase.h"

namespace real_robot_bridge {

class MujocoBackend final : public BackendBase {
 public:
  MujocoBackend(const std::string& xml_file,
                const std::string& simulator_file,
                const MujocoSimulation::RuntimeOptions& runtime_options = MujocoSimulation::RuntimeOptions{});

  std::string name() const override { return "mujoco"; }
  bool supportsGroundTruthContact() const override { return true; }
  double defaultPublishRateHz() const override;
  void prepareForControlStart() override;
  void writeCommand(const legged_msgs::msg::JointControlData& command) override;
  void update() override;
  bool read(BackendData& data) override;

 private:
  rclcpp::Node::SharedPtr sim_node_;
  std::unique_ptr<MujocoSimulation> simulation_;
  double control_period_sec_{0.0};
  double render_period_sec_{0.0};
  double sim_time_reference_{0.0};
  std::chrono::steady_clock::time_point wall_time_reference_;
  std::chrono::steady_clock::time_point last_render_time_;
};

}  // namespace real_robot_bridge
