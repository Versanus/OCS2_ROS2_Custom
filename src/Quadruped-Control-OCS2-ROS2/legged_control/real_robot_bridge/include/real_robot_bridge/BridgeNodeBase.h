#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "legged_msgs/msg/joint_control_data.hpp"
#include "legged_msgs/msg/simulator_sensor_data.hpp"
#include "legged_msgs/msg/simulator_state_data.hpp"
#include "legged_msgs/srv/start_control.hpp"
#include "real_robot_bridge/BackendBase.h"
#include "real_robot_bridge/ContactEstimator.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace real_robot_bridge {

class BridgeNodeBase : public rclcpp::Node {
 public:
  explicit BridgeNodeBase(const std::string& node_name,
                          const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:
  void initializeBackend(std::unique_ptr<BackendBase> backend);

 private:
  void commandCallback(const legged_msgs::msg::JointControlData::SharedPtr msg);
  void publishCallback();
  void startControlService(
      const std::shared_ptr<legged_msgs::srv::StartControl::Request> request,
      std::shared_ptr<legged_msgs::srv::StartControl::Response> response);

  void applyConfiguredContactSource(BackendData& data);
  void logLegacyStatePublish(const legged_msgs::msg::SimulatorStateData& state) const;
  void logLegacyStateResponse(const legged_msgs::msg::SimulatorStateData& state) const;
  void logSensorPublish(const legged_msgs::msg::SimulatorSensorData& sensor) const;
  void overwriteContactFlags(BackendData& data, const std::array<bool, 4>& contact_flags) const;
  void publishEstimatedContactMarkers(const legged_msgs::msg::SimulatorStateData& state,
                                      const std::array<bool, 4>& contact_flags);

  std::unique_ptr<BackendBase> backend_;
  ContactEstimator contact_estimator_;
  ContactSource contact_source_{ContactSource::kEstimated};
  bool state_estimate_{false};
  bool always_publish_state_topic_{false};
  bool debug_state_logging_{true};
  double publish_rate_hz_{200.0};

  rclcpp::Publisher<legged_msgs::msg::SimulatorStateData>::SharedPtr state_pub_;
  rclcpp::Publisher<legged_msgs::msg::SimulatorSensorData>::SharedPtr sensor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr estimated_contact_markers_pub_;
  rclcpp::Subscription<legged_msgs::msg::JointControlData>::SharedPtr joint_command_sub_;
  rclcpp::Service<legged_msgs::srv::StartControl>::SharedPtr start_control_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace real_robot_bridge
