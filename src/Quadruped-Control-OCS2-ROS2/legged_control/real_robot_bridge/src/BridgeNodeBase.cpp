#include "real_robot_bridge/BridgeNodeBase.h"

#include <algorithm>
#include <stdexcept>
#include <string>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace real_robot_bridge {
namespace {

const std::array<const char*, 4> kEstimatedContactFrameNames{{
    "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT",
}};

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
      contact_estimator_() {
  declare_parameter<double>("estimatedContactFilterAlpha", 0.8);
  declare_parameter<double>("estimatedContactOnThreshold", 4.0);
  declare_parameter<double>("estimatedContactOffThreshold", 3.0);
  declare_parameter<std::string>("taskFile", "");
  declare_parameter<std::string>("urdfFile", "");
  declare_parameter<std::string>("contactSource", "mujoco");
  declare_parameter<bool>("alwaysPublishStateTopic", false);
  declare_parameter<bool>("debugStateLogging", true);
  declare_parameter<double>("publishRateHz", 0.0);
}

void BridgeNodeBase::initializeBackend(std::unique_ptr<BackendBase> backend) {
  if (!backend) {
    throw std::runtime_error("Bridge backend cannot be null.");
  }

  backend_ = std::move(backend);

  const auto task_file = get_parameter("taskFile").as_string();
  const auto urdf_file = get_parameter("urdfFile").as_string();
  if (task_file.empty()) {
    throw std::runtime_error("Bridge requires a non-empty 'taskFile' parameter.");
  }

  boost::property_tree::ptree task_info_tree;
  boost::property_tree::read_info(task_file, task_info_tree);
  ContactEstimator::Config estimated_contact_config;
  estimated_contact_config.filter_alpha = get_parameter("estimatedContactFilterAlpha").as_double();
  estimated_contact_config.filter_alpha_rising = estimated_contact_config.filter_alpha;
  estimated_contact_config.filter_alpha_falling = estimated_contact_config.filter_alpha;
  estimated_contact_config.on_threshold = get_parameter("estimatedContactOnThreshold").as_double();
  estimated_contact_config.off_threshold = get_parameter("estimatedContactOffThreshold").as_double();
  estimated_contact_config.filter_alpha = task_info_tree.get<double>(
      "estimatedContact.filterAlpha", estimated_contact_config.filter_alpha);
  estimated_contact_config.filter_alpha_rising = task_info_tree.get<double>(
      "estimatedContact.filterAlphaRise", estimated_contact_config.filter_alpha);
  estimated_contact_config.filter_alpha_falling = task_info_tree.get<double>(
      "estimatedContact.filterAlphaFall", estimated_contact_config.filter_alpha);
  estimated_contact_config.on_threshold = task_info_tree.get<double>(
      "estimatedContact.onThreshold", estimated_contact_config.on_threshold);
  estimated_contact_config.off_threshold = task_info_tree.get<double>(
      "estimatedContact.offThreshold", estimated_contact_config.off_threshold);
  estimated_contact_config.on_confirmation_samples = task_info_tree.get<int>(
      "estimatedContact.onConfirmationSamples", estimated_contact_config.on_confirmation_samples);
  estimated_contact_config.off_confirmation_samples = task_info_tree.get<int>(
      "estimatedContact.offConfirmationSamples", estimated_contact_config.off_confirmation_samples);
  estimated_contact_config.kinematic_max_height = task_info_tree.get<double>(
      "estimatedContact.kinematicMaxHeight", estimated_contact_config.kinematic_max_height);
  estimated_contact_config.kinematic_max_vertical_speed = task_info_tree.get<double>(
      "estimatedContact.kinematicMaxVerticalSpeed", estimated_contact_config.kinematic_max_vertical_speed);
  estimated_contact_config.kinematic_min_liftoff_vertical_speed = task_info_tree.get<double>(
      "estimatedContact.kinematicMinLiftoffVerticalSpeed", estimated_contact_config.kinematic_min_liftoff_vertical_speed);
  estimated_contact_config.strong_force_margin = task_info_tree.get<double>(
      "estimatedContact.strongForceMargin", estimated_contact_config.strong_force_margin);
  contact_estimator_.setConfig(estimated_contact_config);

  ocs2::loadData::loadCppDataType(task_file, "stateEstimate", state_estimate_);
  contact_source_ = parseContactSource(get_parameter("contactSource").as_string());
  always_publish_state_topic_ = get_parameter("alwaysPublishStateTopic").as_bool();
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

  if (contact_source_ == ContactSource::kEstimated) {
    contact_estimator_.initialize(urdf_file);
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
  }
  if (!state_estimate_ || always_publish_state_topic_) {
    state_pub_ = create_publisher<legged_msgs::msg::SimulatorStateData>("simulator_state_data", 1);
  }
  if (contact_source_ == ContactSource::kEstimated) {
    estimated_contact_markers_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("estimated_contact_markers", 1);
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
  if (debug_state_logging_) {
    if (state_estimate_) {
      logSensorPublish(data.sensor);
    } else {
      logLegacyStatePublish(data.state);
    }
  }

  if (sensor_pub_) {
    sensor_pub_->publish(data.sensor);
  }
  if (state_pub_) {
    state_pub_->publish(data.state);
  }
  if (contact_source_ == ContactSource::kEstimated) {
    std::array<bool, 4> contact_flags{};
    for (std::size_t i = 0; i < std::min<std::size_t>(contact_flags.size(), data.sensor.contact_flags.size()); ++i) {
      contact_flags[i] = data.sensor.contact_flags[i];
    }
    publishEstimatedContactMarkers(data.state, contact_flags);
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
  if (debug_state_logging_) {
    logLegacyStateResponse(data.state);
  }
  response->success = true;
  response->state = data.state;
}

void BridgeNodeBase::applyConfiguredContactSource(BackendData& data) {
  if (contact_source_ == ContactSource::kMujoco) {
    return;
  }

  const auto& joint_positions =
      data.sensor.joint_position_values.empty() ? data.state.joint_position_values : data.sensor.joint_position_values;
  const auto& joint_velocities =
      data.sensor.joint_velocity_values.empty() ? data.state.joint_velocity_values : data.sensor.joint_velocity_values;
  const auto estimated_contact_flags = contact_estimator_.update(
      joint_positions, joint_velocities, data.state.joint_torque_values,
      data.state.base_pose_values, data.state.base_quat_values,
      data.state.base_angvel_values, data.state.base_linvel_values);
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

void BridgeNodeBase::logSensorPublish(const legged_msgs::msg::SimulatorSensorData& sensor) const {
  RCLCPP_INFO(get_logger(),
              "Simulation time = [%f], \n"
              "Publishing imu quat data: [%f, %f, %f, %f], \n"
              "Publishing imu angvel data: [%f, %f, %f], \n"
              "Publishing imu linacc data: [%f, %f, %f], \n"
              "Publishing joint position data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
              "Publishing joint velocity data: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f], \n"
              "Publishing contact flag: [%zu, %zu, %zu, %zu]",
              sensor.simulation_time,
              sensor.imu_quat_values[0], sensor.imu_quat_values[1], sensor.imu_quat_values[2], sensor.imu_quat_values[3],
              sensor.imu_angvel_values[0], sensor.imu_angvel_values[1], sensor.imu_angvel_values[2],
              sensor.imu_linacc_values[0], sensor.imu_linacc_values[1], sensor.imu_linacc_values[2],
              sensor.joint_position_values[0], sensor.joint_position_values[1], sensor.joint_position_values[2],
              sensor.joint_position_values[3], sensor.joint_position_values[4], sensor.joint_position_values[5],
              sensor.joint_position_values[6], sensor.joint_position_values[7], sensor.joint_position_values[8],
              sensor.joint_position_values[9], sensor.joint_position_values[10], sensor.joint_position_values[11],
              sensor.joint_velocity_values[0], sensor.joint_velocity_values[1], sensor.joint_velocity_values[2],
              sensor.joint_velocity_values[3], sensor.joint_velocity_values[4], sensor.joint_velocity_values[5],
              sensor.joint_velocity_values[6], sensor.joint_velocity_values[7], sensor.joint_velocity_values[8],
              sensor.joint_velocity_values[9], sensor.joint_velocity_values[10], sensor.joint_velocity_values[11],
              static_cast<size_t>(sensor.contact_flags[0]), static_cast<size_t>(sensor.contact_flags[1]),
              static_cast<size_t>(sensor.contact_flags[2]), static_cast<size_t>(sensor.contact_flags[3]));
}

void BridgeNodeBase::overwriteContactFlags(BackendData& data, const std::array<bool, 4>& contact_flags) const {
  data.state.contact_flags.assign(contact_flags.begin(), contact_flags.end());
  data.sensor.contact_flags.assign(contact_flags.begin(), contact_flags.end());
}

void BridgeNodeBase::publishEstimatedContactMarkers(const legged_msgs::msg::SimulatorStateData& state,
                                                    const std::array<bool, 4>& contact_flags) {
  if (!estimated_contact_markers_pub_) {
    return;
  }

  (void)state;

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(contact_flags.size());

  for (std::size_t i = 0; i < contact_flags.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = kEstimatedContactFrameNames[i];
    marker.ns = "estimated_contact";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = contact_flags[i] ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = -0.025;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = -0.010;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.92f;
    marker.color.g = 0.18f;
    marker.color.b = 0.18f;
    marker.color.a = 0.5f;
    marker_array.markers.push_back(marker);
  }

  estimated_contact_markers_pub_->publish(marker_array);
}

}  // namespace real_robot_bridge
