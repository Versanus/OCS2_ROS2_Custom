#pragma once

#include <string>

#include "legged_msgs/msg/joint_control_data.hpp"
#include "real_robot_bridge/BridgeTypes.h"

namespace real_robot_bridge {

class BackendBase {
 public:
  virtual ~BackendBase() = default;

  virtual std::string name() const = 0;
  virtual bool supportsGroundTruthContact() const { return false; }
  virtual bool hasFullStateEstimate() const { return true; }
  virtual double defaultPublishRateHz() const = 0;

  virtual void prepareForControlStart() {}
  virtual void writeCommand(const legged_msgs::msg::JointControlData& command) = 0;
  virtual void update() = 0;
  virtual bool read(BackendData& data) = 0;
};

}  // namespace real_robot_bridge
