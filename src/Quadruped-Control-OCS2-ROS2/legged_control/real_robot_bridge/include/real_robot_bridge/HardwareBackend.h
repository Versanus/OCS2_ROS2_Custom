#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "real_robot_bridge/BackendBase.h"

namespace real_robot_bridge {

class HardwareBackend final : public BackendBase {
 public:
  explicit HardwareBackend(rclcpp::Node& node);

  std::string name() const override { return "real"; }
  bool hasFullStateEstimate() const override;
  double defaultPublishRateHz() const override { return 250.0; }
  void writeCommand(const legged_msgs::msg::JointControlData& command) override;
  void update() override {}
  bool read(BackendData& data) override;

 private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  static double stampToSeconds(const builtin_interfaces::msg::Time& stamp);
  static std::vector<std::string> defaultJointNames();
  static void assignVectorByJointNames(std::vector<double>& target,
                                       const std::vector<std::string>& target_names,
                                       const sensor_msgs::msg::JointState& source,
                                       const std::vector<double>& source_values);
  static void resizeAndAssign(std::vector<double>& target, const std::vector<double>& source, std::size_t size);

  rclcpp::Node& node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr hardware_command_pub_;

  std::string odom_topic_;
  double default_base_height_{0.0};
  std::vector<std::string> configured_joint_names_;

  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  sensor_msgs::msg::Imu latest_imu_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_joint_state_{false};
  bool have_imu_{false};
  bool have_odom_{false};
};

}  // namespace real_robot_bridge
