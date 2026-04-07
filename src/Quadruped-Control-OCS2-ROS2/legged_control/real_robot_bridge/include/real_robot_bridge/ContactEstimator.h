#pragma once

#include <array>
#include <vector>

namespace real_robot_bridge {

class ContactEstimator {
 public:
  struct Config {
    double filter_alpha = 0.85;
    double on_threshold = 6.0;
    double off_threshold = 3.0;
  };

  ContactEstimator();
  explicit ContactEstimator(const Config& config);

  std::array<bool, 4> update(const std::vector<double>& joint_torques);

 private:
  Config config_;
  std::array<double, 4> filtered_leg_loads_{};
  std::array<bool, 4> contact_flags_{};
};

}  // namespace real_robot_bridge
