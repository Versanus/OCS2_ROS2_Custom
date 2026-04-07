#include "real_robot_bridge/BridgeNodeBase.h"

#include <algorithm>
#include <stdexcept>
#include <string>

#include <ocs2_core/misc/LoadData.h>

namespace real_robot_bridge {
namespace {

ContactSource parseContactSource(const std::string& value) {
  if (value == "mujoco") {
    return ContactSource::kMujoco;
  }
  if (value == "estimated") {
    return ContactSource::kEstimated;
  }

  throw std::runtime_error("Unsupported contactSource '" + value + "'. Expected 'mujoco' or 'estimated'.");
}

std::chrono::nanoseconds periodFromHz(double hz) {
  const double clamped_hz = std::max(1.0, hz);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / clamped_hz));
}

}  // namespace

BridgeNodeBase::BridgeNodeBase(const std::string& node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options),
      contact_estimator_(ContactEstimator::Config{
          declare_parameter<double>("estimatedContactFilterAlpha", 0.85),
          declare_parameter<double>("estimatedContactOnThreshold", 6.0),
          declare_parameter<double>("estimatedContactOffThreshold", 3.0),
      }) {
  declare_parameter<std::string>("taskFile", "");
  declare_parameter<std::string>("contactSource", "mujoco");
  declare_parameter<bool>("debugStateLogging", true);
  declare_parameter<double>("publishRateHz", 0.0);
}

void BridgeNodeBase::initializeBackend(std::unique_ptr<BackendBase> backend) {
  if (!backend) {
    throw std::runtime_error("Bridge backend cannot be null.");
  }

  backend_ = std::move(backend);

  const auto task_file = get_parameter("taskFile").as_string();
  if (task_file.empty()) {
    throw std::runtime_error("Bridge requires a non-empty 'taskFile' parameter.");
  }

  ocs2::loadData::loadCppDataType(task_file, "stateEstimate", state_estimate_);
  contact_source_ = parseContactSource(get_parameter("contactSource").as_string());
  debug_state_logging_ = get_parameter("debugStateLogging").as_bool();
  publish_rate_hz_ = get_parameter("publishRateHz").as_double();
  if (publish_rate_hz_ <= 0.0) {
    publish_rate_hz_ = backend_->defaultPublishRateHz();
  }

  if (contact_source_ == ContactSource::kMujoco && !backend_->supportsGroundTruthContact()) {
    RCLCPP_WARN(get_logger(),
                "Backend '%s' has no direct MuJoCo contact source. Falling back to estimated contacts.",
                backend_->name().c_str());
    contact_source_ = ContactSource::kEstimated;
  }

  if (!state_estimate_ && !backend_->hasFullStateEstimate()) {
    throw std::runtime_error(
        "stateEstimate=false requires a backend that can provide full base pose/velocity. "
        "Configure a full-state source for the selected backend or switch task.info to stateEstimate=true.");
  }

  joint_command_sub_ = create_subscription<legged_msgs::msg::JointControlData>(
      "joint_control_data", 1, std::bind(&BridgeNodeBase::commandCallback, this, std::placeholders::_1));
  start_control_srv_ = create_service<legged_msgs::srv::StartControl>(
      "start_control", std::bind(&BridgeNodeBase::startControlService, this, std::placeholders::_1, std::placeholders::_2));

  if (state_estimate_) {
    sensor_pub_ = create_publisher<legged_msgs::msg::SimulatorSensorData>("simulator_sensor_data", 1);
  } else {
    state_pub_ = create_publisher<legged_msgs::msg::SimulatorStateData>("simulator_state_data", 1);
  }

  publish_timer_ = create_wall_timer(periodFromHz(publish_rate_hz_), std::bind(&BridgeNodeBase::publishCallback, this));

  RCLCPP_INFO(get_logger(),
              "Initialized bridge backend='%s' stateEstimate=%s contactSource=%s publishRateHz=%.2f",
              backend_->name().c_str(), state_estimate_ ? "true" : "false", toString(contact_source_), publish_rate_hz_);
}

void BridgeNodeBase::commandCallback(const legged_msgs::msg::JointControlData::SharedPtr msg) {
  if (!backend_) {
    return;
  }
  backend_->writeCommand(*msg);
}

void BridgeNodeBase::publishCallback() {
  if (!backend_) {
    return;
  }

  backend_->update();

  BackendData data;
  if (!backend_->read(data)) {
    return;
  }

  applyConfiguredContactSource(data);
  if (debug_state_logging_ && !state_estimate_) {
    logLegacyStatePublish(data.state);
  }

  if (state_estimate_) {
    if (sensor_pub_) {
      sensor_pub_->publish(data.sensor);
    }
  } else {
    if (state_pub_) {
      state_pub_->publish(data.state);
    }
  }
}

void BridgeNodeBase::startControlService(
    const std::shared_ptr<legged_msgs::srv::StartControl::Request> request,
    std::shared_ptr<legged_msgs::srv::StartControl::Response> response) {
  if (!backend_) {
    response->success = false;
    return;
  }

  (void)request;
  backend_->prepareForControlStart();

  BackendData data;
  if (!backend_->read(data)) {
    response->success = false;
    return;
  }

  applyConfiguredContactSource(data);
  if (debug_state_logging_ && !state_estimate_) {
    logLegacyStateResponse(data.state);
  }
  response->success = true;
  response->state = data.state;
}

void BridgeNodeBase::applyConfiguredContactSource(BackendData& data) {
  if (contact_source_ == ContactSource::kMujoco) {
    return;
  }

  const auto estimated_contact_flags = contact_estimator_.update(data.state.joint_torque_values);
  overwriteContactFlags(data, estimated_contact_flags);
}

void BridgeNodeBase::logLegacyStatePublish(const legged_msgs::msg::SimulatorStateData& state) const {
  RCLCPP_INFO(get_logger(),
              "Simulation time = [%f], \n"
              "Publishing base quat data: [%f, %f, %f, %f], \n"
              "Publishing base pos data: [%f, %f, %f], \n"
              "Publishing base angvel data: [%f, %f, %f], \n"
              "Publishing base linvel data: [%f, %f, %f], \n"
              "Publishing joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
              "Publishing joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
              "Publishing joint torque data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
              "Publishing contact flag: [%zu, %zu, %zu, %zu]",
              state.simulation_time,
              state.base_quat_values[0], state.base_quat_values[1], state.base_quat_values[2], state.base_quat_values[3],
              state.base_pose_values[0], state.base_pose_values[1], state.base_pose_values[2],
              state.base_angvel_values[0], state.base_angvel_values[1], state.base_angvel_values[2],
              state.base_linvel_values[0], state.base_linvel_values[1], state.base_linvel_values[2],
              state.joint_position_values[0], state.joint_position_values[1], state.joint_position_values[2], state.joint_position_values[3],
              state.joint_position_values[4], state.joint_position_values[5], state.joint_position_values[6], state.joint_position_values[7],
              state.joint_position_values[8], state.joint_position_values[9], state.joint_position_values[10], state.joint_position_values[11],
              state.joint_velocity_values[0], state.joint_velocity_values[1], state.joint_velocity_values[2], state.joint_velocity_values[3],
              state.joint_velocity_values[4], state.joint_velocity_values[5], state.joint_velocity_values[6], state.joint_velocity_values[7],
              state.joint_velocity_values[8], state.joint_velocity_values[9], state.joint_velocity_values[10], state.joint_velocity_values[11],
              state.joint_torque_values[0], state.joint_torque_values[1], state.joint_torque_values[2], state.joint_torque_values[3],
              state.joint_torque_values[4], state.joint_torque_values[5], state.joint_torque_values[6], state.joint_torque_values[7],
              state.joint_torque_values[8], state.joint_torque_values[9], state.joint_torque_values[10], state.joint_torque_values[11],
              static_cast<size_t>(state.contact_flags[0]), static_cast<size_t>(state.contact_flags[1]),
              static_cast<size_t>(state.contact_flags[2]), static_cast<size_t>(state.contact_flags[3]));
  RCLCPP_INFO(get_logger(), "Simulation time = [%f]", state.simulation_time);
}

void BridgeNodeBase::logLegacyStateResponse(const legged_msgs::msg::SimulatorStateData& state) const {
  RCLCPP_INFO(get_logger(),
              "Simulation time = [%f], \n"
              "Responding base quat data: [%f, %f, %f, %f], \n"
              "Responding base pos data: [%f, %f, %f], \n"
              "Responding base angvel data: [%f, %f, %f], \n"
              "Responding base linvel data: [%f, %f, %f], \n"
              "Responding joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
              "Responding joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
              "Responding joint torque data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
              "Responding contact flag: [%zu, %zu, %zu, %zu]",
              state.simulation_time,
              state.base_quat_values[0], state.base_quat_values[1], state.base_quat_values[2], state.base_quat_values[3],
              state.base_pose_values[0], state.base_pose_values[1], state.base_pose_values[2],
              state.base_angvel_values[0], state.base_angvel_values[1], state.base_angvel_values[2],
              state.base_linvel_values[0], state.base_linvel_values[1], state.base_linvel_values[2],
              state.joint_position_values[0], state.joint_position_values[1], state.joint_position_values[2], state.joint_position_values[3],
              state.joint_position_values[4], state.joint_position_values[5], state.joint_position_values[6], state.joint_position_values[7],
              state.joint_position_values[8], state.joint_position_values[9], state.joint_position_values[10], state.joint_position_values[11],
              state.joint_velocity_values[0], state.joint_velocity_values[1], state.joint_velocity_values[2], state.joint_velocity_values[3],
              state.joint_velocity_values[4], state.joint_velocity_values[5], state.joint_velocity_values[6], state.joint_velocity_values[7],
              state.joint_velocity_values[8], state.joint_velocity_values[9], state.joint_velocity_values[10], state.joint_velocity_values[11],
              state.joint_torque_values[0], state.joint_torque_values[1], state.joint_torque_values[2], state.joint_torque_values[3],
              state.joint_torque_values[4], state.joint_torque_values[5], state.joint_torque_values[6], state.joint_torque_values[7],
              state.joint_torque_values[8], state.joint_torque_values[9], state.joint_torque_values[10], state.joint_torque_values[11],
              static_cast<size_t>(state.contact_flags[0]), static_cast<size_t>(state.contact_flags[1]),
              static_cast<size_t>(state.contact_flags[2]), static_cast<size_t>(state.contact_flags[3]));
}

void BridgeNodeBase::overwriteContactFlags(BackendData& data, const std::array<bool, 4>& contact_flags) const {
  data.state.contact_flags.assign(contact_flags.begin(), contact_flags.end());
  data.sensor.contact_flags.assign(contact_flags.begin(), contact_flags.end());
}

}  // namespace real_robot_bridge
