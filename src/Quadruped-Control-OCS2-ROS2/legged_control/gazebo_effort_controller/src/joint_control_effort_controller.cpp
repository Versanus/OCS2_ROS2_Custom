#include "gazebo_effort_controller/joint_control_effort_controller.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace
{
constexpr const char* kEffortInterface = "effort";
constexpr const char* kPositionInterface = "position";
constexpr const char* kVelocityInterface = "velocity";

double sanitizeGain(double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::max(0.0, value);
}
}  // namespace

namespace gazebo_effort_controller
{

controller_interface::CallbackReturn JointControlEffortController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<double>("base_kp", base_kp_);
    auto_declare<double>("base_kd", base_kd_);
    auto_declare<std::string>("command_topic", command_topic_);
    auto_declare<std::string>("output_joint_state_topic", output_joint_state_topic_);
  } catch (const std::exception& error) {
    fprintf(stderr, "JointControlEffortController parameter declaration failed: %s\n", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointControlEffortController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + kEffortInterface);
  }

  return config;
}

controller_interface::InterfaceConfiguration JointControlEffortController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint_name : joint_names_) {
    config.names.push_back(joint_name + "/" + kPositionInterface);
    config.names.push_back(joint_name + "/" + kVelocityInterface);
    config.names.push_back(joint_name + "/" + kEffortInterface);
  }

  return config;
}

controller_interface::CallbackReturn JointControlEffortController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  base_kp_ = get_node()->get_parameter("base_kp").as_double();
  base_kd_ = get_node()->get_parameter("base_kd").as_double();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  output_joint_state_topic_ = get_node()->get_parameter("output_joint_state_topic").as_string();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  latest_command_.joint_torque.assign(joint_names_.size(), 0.0);
  latest_command_.joint_position.assign(joint_names_.size(), 0.0);
  latest_command_.joint_velocity.assign(joint_names_.size(), 0.0);
  latest_command_.kp = 0.0;
  latest_command_.kd = 0.0;

  command_subscriber_ = get_node()->create_subscription<legged_msgs::msg::JointControlData>(
      command_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&JointControlEffortController::commandCallback, this, std::placeholders::_1));

  joint_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      output_joint_state_topic_, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured JointControlData effort controller: command=%s state=%s base_kp=%.3f base_kd=%.3f joints=%zu",
              command_topic_.c_str(), output_joint_state_topic_.c_str(), base_kp_, base_kd_, joint_names_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointControlEffortController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  has_command_ = false;
  latest_command_.joint_torque.assign(joint_names_.size(), 0.0);
  latest_command_.joint_position.assign(joint_names_.size(), 0.0);
  latest_command_.joint_velocity.assign(joint_names_.size(), 0.0);
  latest_command_.kp = 0.0;
  latest_command_.kd = 0.0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointControlEffortController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  const auto joint_count = joint_names_.size();
  std::vector<double> positions(joint_count, 0.0);
  std::vector<double> velocities(joint_count, 0.0);
  std::vector<double> efforts(joint_count, 0.0);

  for (size_t index = 0; index < joint_count; ++index) {
    const size_t state_offset = index * 3;
    positions[index] = state_interfaces_[state_offset].get_value();
    velocities[index] = state_interfaces_[state_offset + 1].get_value();
    efforts[index] = state_interfaces_[state_offset + 2].get_value();
  }

  legged_msgs::msg::JointControlData command;
  if (!copyLatestCommand(command)) {
    publishJointState(time, positions, velocities, efforts);
    return controller_interface::return_type::OK;
  }

  const double kp = base_kp_ * sanitizeGain(command.kp);
  const double kd = base_kd_ * sanitizeGain(command.kd);

  for (size_t index = 0; index < joint_count; ++index) {
    const double effort = command.joint_torque[index]
        + kp * (command.joint_position[index] - positions[index])
        + kd * (command.joint_velocity[index] - velocities[index]);
    command_interfaces_[index].set_value(effort);
  }

  publishJointState(time, positions, velocities, efforts);
  return controller_interface::return_type::OK;
}

void JointControlEffortController::commandCallback(const legged_msgs::msg::JointControlData::SharedPtr msg)
{
  const auto expected_size = joint_names_.size();
  if (msg->joint_torque.size() != expected_size
      || msg->joint_position.size() != expected_size
      || msg->joint_velocity.size() != expected_size) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring JointControlData with invalid sizes: torque=%zu position=%zu velocity=%zu expected=%zu",
                msg->joint_torque.size(), msg->joint_position.size(), msg->joint_velocity.size(), expected_size);
    return;
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  latest_command_ = *msg;
  has_command_ = true;
}

bool JointControlEffortController::copyLatestCommand(legged_msgs::msg::JointControlData& command) const
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  if (!has_command_) {
    return false;
  }
  command = latest_command_;
  return true;
}

void JointControlEffortController::publishJointState(const rclcpp::Time& stamp,
                                                     const std::vector<double>& positions,
                                                     const std::vector<double>& velocities,
                                                     const std::vector<double>& efforts)
{
  if (!joint_state_publisher_) {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = joint_names_;
  msg.position = positions;
  msg.velocity = velocities;
  msg.effort = efforts;
  joint_state_publisher_->publish(msg);
}

}  // namespace gazebo_effort_controller

PLUGINLIB_EXPORT_CLASS(gazebo_effort_controller::JointControlEffortController,
                       controller_interface::ControllerInterface)
