#pragma once

#include "legged_msgs/msg/simulator_sensor_data.hpp"
#include "legged_msgs/msg/simulator_state_data.hpp"

namespace real_robot_bridge {

enum class ContactSource {
  kMujoco,
  kEstimated,
};

struct BackendData {
  legged_msgs::msg::SimulatorStateData state;
  legged_msgs::msg::SimulatorSensorData sensor;
};

inline const char* toString(ContactSource source) {
  switch (source) {
    case ContactSource::kMujoco:
      return "mujoco";
    case ContactSource::kEstimated:
      return "estimated";
    default:
      return "unknown";
  }
}

}  // namespace real_robot_bridge
