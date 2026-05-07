#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "legged_msgs/msg/joint_control_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace gazebo_effort_controller
{

class JointControlEffortController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void commandCallback(const legged_msgs::msg::JointControlData::SharedPtr msg);
  bool copyLatestCommand(legged_msgs::msg::JointControlData& command) const;
  void publishJointState(const rclcpp::Time& stamp,
                         const std::vector<double>& positions,
                         const std::vector<double>& velocities,
                         const std::vector<double>& efforts);

  std::vector<std::string> joint_names_;
  double base_kp_ = 10.0;
  double base_kd_ = 0.30;
  std::string command_topic_ = "/joint_control_data";
  std::string output_joint_state_topic_ = "/joint_states";

  rclcpp::Subscription<legged_msgs::msg::JointControlData>::SharedPtr command_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

  mutable std::mutex command_mutex_;
  legged_msgs::msg::JointControlData latest_command_;
  bool has_command_ = false;
};

}  // namespace gazebo_effort_controller
