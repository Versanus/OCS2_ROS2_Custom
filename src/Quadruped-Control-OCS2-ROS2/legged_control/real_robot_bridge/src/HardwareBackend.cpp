#include "real_robot_bridge/HardwareBackend.h"

#include <algorithm>

namespace real_robot_bridge {

HardwareBackend::HardwareBackend(rclcpp::Node& node) : node_(node) {
  const auto joint_state_topic = node_.declare_parameter<std::string>("jointStateTopic", "joint_states");
  const auto imu_topic = node_.declare_parameter<std::string>("imuTopic", "imu/data");
  odom_topic_ = node_.declare_parameter<std::string>("odomTopic", "");
  const auto hardware_command_topic =
      node_.declare_parameter<std::string>("hardwareCommandTopic", "htdw_joint_cmd");
  configured_joint_names_ =
      node_.declare_parameter<std::vector<std::string>>("hardwareJointNames", defaultJointNames());
  default_base_height_ = node_.declare_parameter<double>("defaultBaseHeight", 0.0);

  joint_state_sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10, std::bind(&HardwareBackend::jointStateCallback, this, std::placeholders::_1));
  imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 10, std::bind(&HardwareBackend::imuCallback, this, std::placeholders::_1));

  if (!odom_topic_.empty()) {
    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&HardwareBackend::odomCallback, this, std::placeholders::_1));
  }

  hardware_command_pub_ = node_.create_publisher<trajectory_msgs::msg::JointTrajectory>(hardware_command_topic, 10);
}

bool HardwareBackend::hasFullStateEstimate() const {
  return !odom_topic_.empty();
}

void HardwareBackend::writeCommand(const legged_msgs::msg::JointControlData& command) {
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = node_.now();

  {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (!latest_joint_state_.name.empty()) {
      trajectory.joint_names = latest_joint_state_.name;
    } else {
      trajectory.joint_names = configured_joint_names_;
    }
  }

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = command.joint_position;
  point.velocities = command.joint_velocity;
  point.effort = command.joint_torque;
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
  const double now_time = node_.now().seconds();
  const double simulation_time = std::max({joint_time, imu_time, odom_time, now_time});

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

  latest_joint_state_.header = msg->header;
  latest_joint_state_.name = configured_joint_names_;
  assignVectorByJointNames(latest_joint_state_.position, configured_joint_names_, *msg, msg->position);
  assignVectorByJointNames(latest_joint_state_.velocity, configured_joint_names_, *msg, msg->velocity);
  assignVectorByJointNames(latest_joint_state_.effort, configured_joint_names_, *msg, msg->effort);
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
      if (target_names[target_index] == source.name[source_index] && source_index < source_values.size()) {
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
