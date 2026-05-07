#include "real_robot_bridge/HardwareBackend.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace real_robot_bridge {
namespace {

template <typename Accessor>
bool extractTrajectoryField(const trajectory_msgs::msg::JointTrajectory& source,
                            std::size_t expected_size,
                            Accessor accessor,
                            std::vector<double>& target) {
  target.assign(expected_size, 0.0);
  if (source.points.empty()) {
    return false;
  }

  const auto& first_values = accessor(source.points.front());
  if (first_values.size() >= expected_size) {
    std::copy_n(first_values.begin(), expected_size, target.begin());
    return true;
  }

  bool filled = false;
  const auto copy_count = std::min(expected_size, source.points.size());
  for (std::size_t index = 0; index < copy_count; ++index) {
    const auto& values = accessor(source.points[index]);
    if (!values.empty()) {
      target[index] = values.front();
      filled = true;
    }
  }

  return filled;
}

std::string compactUppercase(const std::string& input) {
  std::string result;
  result.reserve(input.size());

  for (const unsigned char ch : input) {
    if (ch == '-' || ch == ' ' || ch == '_') {
      continue;
    }
    result.push_back(static_cast<char>(std::toupper(ch)));
  }

  return result;
}

}  // namespace

HardwareBackend::HardwareBackend(rclcpp::Node& node) : node_(node) {
  const auto joint_state_topic = node_.declare_parameter<std::string>("jointStateTopic", "joint_states");
  const auto joint_feedback_source = node_.declare_parameter<std::string>("jointFeedbackSource", "joint_state");
  const auto joint_feedback_topic = node_.declare_parameter<std::string>("jointFeedbackTopic", "htdw_joint_cmd");
  const auto imu_topic = node_.declare_parameter<std::string>("imuTopic", "imu/data");
  odom_topic_ = node_.declare_parameter<std::string>("odomTopic", "");
  const auto hardware_command_topic =
      node_.declare_parameter<std::string>("hardwareCommandTopic", "htdw_joint_cmd");
  configured_joint_names_ =
      node_.declare_parameter<std::vector<std::string>>("hardwareJointNames", defaultJointNames());
  configured_command_joint_names_ =
      node_.declare_parameter<std::vector<std::string>>("commandJointNames", defaultCommandJointNames());
  default_base_height_ = node_.declare_parameter<double>("defaultBaseHeight", 0.0);

  if (joint_feedback_source == "joint_state") {
    joint_state_sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic, 10, std::bind(&HardwareBackend::jointStateCallback, this, std::placeholders::_1));
  } else if (joint_feedback_source == "joint_trajectory") {
    joint_trajectory_sub_ = node_.create_subscription<trajectory_msgs::msg::JointTrajectory>(
        joint_feedback_topic, 10, std::bind(&HardwareBackend::jointTrajectoryCallback, this, std::placeholders::_1));
  } else {
    throw std::runtime_error(
        "Unsupported jointFeedbackSource '" + joint_feedback_source +
        "'. Expected 'joint_state' or 'joint_trajectory'.");
  }

  imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 10, std::bind(&HardwareBackend::imuCallback, this, std::placeholders::_1));

  if (!odom_topic_.empty()) {
    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&HardwareBackend::odomCallback, this, std::placeholders::_1));
  }

  hardware_command_pub_ = node_.create_publisher<trajectory_msgs::msg::JointTrajectory>(hardware_command_topic, 10);

  if (joint_feedback_source == "joint_trajectory" && joint_feedback_topic == hardware_command_topic) {
    RCLCPP_WARN(node_.get_logger(),
                "jointFeedbackTopic and hardwareCommandTopic are both '%s'. "
                "Use separate topics to avoid reading bridge commands back as sensor feedback.",
                joint_feedback_topic.c_str());
  }
}

bool HardwareBackend::hasFullStateEstimate() const {
  return !odom_topic_.empty();
}

void HardwareBackend::writeCommand(const legged_msgs::msg::JointControlData& command) {
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = node_.now();

  {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (!latest_command_joint_names_.empty()) {
      trajectory.joint_names = latest_command_joint_names_;
    } else if (!configured_command_joint_names_.empty()) {
      trajectory.joint_names = configured_command_joint_names_;
    } else {
      trajectory.joint_names = configured_joint_names_;
    }
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = remapVectorByJointNames(command.joint_position, configured_joint_names_, trajectory.joint_names);
  point.velocities = remapVectorByJointNames(command.joint_velocity, configured_joint_names_, trajectory.joint_names);
  point.effort = remapVectorByJointNames(command.joint_torque, configured_joint_names_, trajectory.joint_names);
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 0;
  trajectory.points.push_back(point);

  hardware_command_pub_->publish(trajectory);
}

bool HardwareBackend::read(BackendData& data) {
  std::scoped_lock<std::mutex> lock(mutex_);

  if (!have_joint_state_ || !have_imu_ || (!odom_topic_.empty() && !have_odom_)) {
    return false;
  }

  const double joint_time = stampToSeconds(latest_joint_state_.header.stamp);
  const double imu_time = stampToSeconds(latest_imu_.header.stamp);
  const double odom_time = have_odom_ ? stampToSeconds(latest_odom_.header.stamp) : 0.0;
  double simulation_time = std::max({joint_time, imu_time, odom_time});
  if (simulation_time <= 0.0) {
    // Fall back to the node clock only when sensor timestamps are unavailable.
    simulation_time = node_.now().seconds();
  }

  data.state.simulation_time = simulation_time;
  data.sensor.simulation_time = simulation_time;

  data.state.base_quat_values = {
      latest_imu_.orientation.w,
      latest_imu_.orientation.x,
      latest_imu_.orientation.y,
      latest_imu_.orientation.z,
  };
  data.sensor.imu_quat_values = data.state.base_quat_values;

  data.state.base_angvel_values = {
      latest_imu_.angular_velocity.x,
      latest_imu_.angular_velocity.y,
      latest_imu_.angular_velocity.z,
  };
  data.sensor.imu_angvel_values = data.state.base_angvel_values;

  data.sensor.imu_linacc_values = {
      latest_imu_.linear_acceleration.x,
      latest_imu_.linear_acceleration.y,
      latest_imu_.linear_acceleration.z,
  };

  if (have_odom_) {
    data.state.base_pose_values = {
        latest_odom_.pose.pose.position.x,
        latest_odom_.pose.pose.position.y,
        latest_odom_.pose.pose.position.z,
    };
    data.state.base_linvel_values = {
        latest_odom_.twist.twist.linear.x,
        latest_odom_.twist.twist.linear.y,
        latest_odom_.twist.twist.linear.z,
    };
  } else {
    data.state.base_pose_values = {0.0, 0.0, default_base_height_};
    data.state.base_linvel_values = {0.0, 0.0, 0.0};
  }

  resizeAndAssign(data.state.joint_position_values, latest_joint_state_.position, 12);
  resizeAndAssign(data.state.joint_velocity_values, latest_joint_state_.velocity, 12);
  resizeAndAssign(data.state.joint_torque_values, latest_joint_state_.effort, 12);

  data.sensor.joint_position_values = data.state.joint_position_values;
  data.sensor.joint_velocity_values = data.state.joint_velocity_values;

  data.state.contact_flags.assign(4, false);
  data.sensor.contact_flags.assign(4, false);
  return true;
}

void HardwareBackend::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(mutex_);
  updateLatestJointState(*msg);
}

void HardwareBackend::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  if (msg->points.empty()) {
    RCLCPP_WARN(node_.get_logger(), "Ignoring %s because it does not contain any trajectory points.",
                msg->header.frame_id.c_str());
    return;
  }

  sensor_msgs::msg::JointState joint_state;
  joint_state.header = msg->header;
  joint_state.name = msg->joint_names;

  const auto expected_size = joint_state.name.empty() ? configured_joint_names_.size() : joint_state.name.size();
  if (!extractTrajectoryField(*msg, expected_size, [](const auto& point) -> const auto& { return point.positions; },
                              joint_state.position)) {
    RCLCPP_WARN(node_.get_logger(),
                "Ignoring joint feedback on '%s' because the trajectory message does not contain usable positions.",
                msg->header.frame_id.c_str());
    return;
  }

  if (!extractTrajectoryField(*msg, expected_size, [](const auto& point) -> const auto& { return point.velocities; },
                              joint_state.velocity) &&
      !warned_missing_trajectory_velocity_) {
    RCLCPP_WARN(node_.get_logger(),
                "Joint feedback trajectory is missing velocity data. Filling simulator_sensor_data velocities with zeros.");
    warned_missing_trajectory_velocity_ = true;
  }

  if (!extractTrajectoryField(*msg, expected_size, [](const auto& point) -> const auto& { return point.effort; },
                              joint_state.effort) &&
      !warned_missing_trajectory_effort_) {
    RCLCPP_WARN(node_.get_logger(),
                "Joint feedback trajectory is missing effort data. Estimated contacts will stay unreliable until efforts are present.");
    warned_missing_trajectory_effort_ = true;
  }

  std::scoped_lock<std::mutex> lock(mutex_);
  updateLatestJointState(joint_state);
}

void HardwareBackend::updateLatestJointState(const sensor_msgs::msg::JointState& msg) {
  latest_joint_state_.header = msg.header;
  latest_joint_state_.name = configured_joint_names_;
  assignVectorByJointNames(latest_joint_state_.position, configured_joint_names_, msg, msg.position);
  assignVectorByJointNames(latest_joint_state_.velocity, configured_joint_names_, msg, msg.velocity);
  assignVectorByJointNames(latest_joint_state_.effort, configured_joint_names_, msg, msg.effort);

  latest_command_joint_names_.clear();
  if (!msg.name.empty()) {
    latest_command_joint_names_.reserve(msg.name.size());
    for (const auto& joint_name : msg.name) {
      latest_command_joint_names_.push_back(toHardwareJointName(joint_name));
    }
  }
  if (latest_command_joint_names_.empty()) {
    latest_command_joint_names_ = configured_command_joint_names_;
  } else if (latest_command_joint_names_.size() != configured_command_joint_names_.size()) {
    latest_command_joint_names_ = configured_command_joint_names_;
  }

  have_joint_state_ = true;
}

void HardwareBackend::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(mutex_);
  latest_imu_ = *msg;
  have_imu_ = true;
}

void HardwareBackend::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::scoped_lock<std::mutex> lock(mutex_);
  latest_odom_ = *msg;
  have_odom_ = true;
}

double HardwareBackend::stampToSeconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::vector<std::string> HardwareBackend::defaultJointNames() {
  return {
      "LF_HAA", "LF_HFE", "LF_KFE",
      "LH_HAA", "LH_HFE", "LH_KFE",
      "RF_HAA", "RF_HFE", "RF_KFE",
      "RH_HAA", "RH_HFE", "RH_KFE",
  };
}

std::vector<std::string> HardwareBackend::defaultCommandJointNames() {
  return {
      "FL_HipX", "FL_HipY", "FL_Knee",
      "FR_HipX", "FR_HipY", "FR_Knee",
      "HL_HipX", "HL_HipY", "HL_Knee",
      "HR_HipX", "HR_HipY", "HR_Knee",
  };
}

std::string HardwareBackend::normalizeJointName(const std::string& name) {
  const auto compact_name = compactUppercase(name);

  if (compact_name == "FLHIPX") return "FL_HipX";
  if (compact_name == "FLHIPY") return "FL_HipY";
  if (compact_name == "FLKNEE") return "FL_Knee";
  if (compact_name == "FRHIPX") return "FR_HipX";
  if (compact_name == "FRHIPY") return "FR_HipY";
  if (compact_name == "FRKNEE") return "FR_Knee";
  if (compact_name == "HLHIPX") return "HL_HipX";
  if (compact_name == "HLHIPY") return "HL_HipY";
  if (compact_name == "HLKNEE") return "HL_Knee";
  if (compact_name == "HRHIPX") return "HR_HipX";
  if (compact_name == "HRHIPY") return "HR_HipY";
  if (compact_name == "HRKNEE") return "HR_Knee";

  if (compact_name == "LFHAA") return "LF_HAA";
  if (compact_name == "LFHFE") return "LF_HFE";
  if (compact_name == "LFKFE") return "LF_KFE";
  if (compact_name == "LHHAA") return "LH_HAA";
  if (compact_name == "LHHFE") return "LH_HFE";
  if (compact_name == "LHKFE") return "LH_KFE";
  if (compact_name == "RFHAA") return "RF_HAA";
  if (compact_name == "RFHFE") return "RF_HFE";
  if (compact_name == "RFKFE") return "RF_KFE";
  if (compact_name == "RHHAA") return "RH_HAA";
  if (compact_name == "RHHFE") return "RH_HFE";
  if (compact_name == "RHKFE") return "RH_KFE";

  return name;
}

std::string HardwareBackend::toLeggedJointName(const std::string& name) {
  const auto normalized_name = normalizeJointName(name);

  if (normalized_name == "FL_HipX") return "LF_HAA";
  if (normalized_name == "FL_HipY") return "LF_HFE";
  if (normalized_name == "FL_Knee") return "LF_KFE";
  if (normalized_name == "FR_HipX") return "RF_HAA";
  if (normalized_name == "FR_HipY") return "RF_HFE";
  if (normalized_name == "FR_Knee") return "RF_KFE";
  if (normalized_name == "HL_HipX") return "LH_HAA";
  if (normalized_name == "HL_HipY") return "LH_HFE";
  if (normalized_name == "HL_Knee") return "LH_KFE";
  if (normalized_name == "HR_HipX") return "RH_HAA";
  if (normalized_name == "HR_HipY") return "RH_HFE";
  if (normalized_name == "HR_Knee") return "RH_KFE";

  return normalized_name;
}

std::string HardwareBackend::toHardwareJointName(const std::string& name) {
  const auto normalized_name = normalizeJointName(name);

  if (normalized_name == "LF_HAA") return "FL_HipX";
  if (normalized_name == "LF_HFE") return "FL_HipY";
  if (normalized_name == "LF_KFE") return "FL_Knee";
  if (normalized_name == "RF_HAA") return "FR_HipX";
  if (normalized_name == "RF_HFE") return "FR_HipY";
  if (normalized_name == "RF_KFE") return "FR_Knee";
  if (normalized_name == "LH_HAA") return "HL_HipX";
  if (normalized_name == "LH_HFE") return "HL_HipY";
  if (normalized_name == "LH_KFE") return "HL_Knee";
  if (normalized_name == "RH_HAA") return "HR_HipX";
  if (normalized_name == "RH_HFE") return "HR_HipY";
  if (normalized_name == "RH_KFE") return "HR_Knee";

  return normalized_name;
}

std::vector<double> HardwareBackend::remapVectorByJointNames(const std::vector<double>& source_values,
                                                             const std::vector<std::string>& source_names,
                                                             const std::vector<std::string>& target_names) {
  std::vector<double> remapped_values(target_names.size(), 0.0);

  for (std::size_t target_index = 0; target_index < target_names.size(); ++target_index) {
    const auto target_legged_name = toLeggedJointName(target_names[target_index]);
    for (std::size_t source_index = 0; source_index < source_names.size(); ++source_index) {
      if (target_legged_name == toLeggedJointName(source_names[source_index]) && source_index < source_values.size()) {
        remapped_values[target_index] = source_values[source_index];
        break;
      }
    }
  }

  return remapped_values;
}

void HardwareBackend::assignVectorByJointNames(std::vector<double>& target,
                                               const std::vector<std::string>& target_names,
                                               const sensor_msgs::msg::JointState& source,
                                               const std::vector<double>& source_values) {
  target.assign(target_names.size(), 0.0);

  if (source.name.empty()) {
    resizeAndAssign(target, source_values, target_names.size());
    return;
  }

  for (std::size_t target_index = 0; target_index < target_names.size(); ++target_index) {
    for (std::size_t source_index = 0; source_index < source.name.size(); ++source_index) {
      if (toLeggedJointName(target_names[target_index]) == toLeggedJointName(source.name[source_index]) &&
          source_index < source_values.size()) {
        target[target_index] = source_values[source_index];
        break;
      }
    }
  }
}

void HardwareBackend::resizeAndAssign(std::vector<double>& target, const std::vector<double>& source, std::size_t size) {
  target.assign(size, 0.0);
  const auto copy_count = std::min(size, source.size());
  std::copy_n(source.begin(), copy_count, target.begin());
}

}  // namespace real_robot_bridge
