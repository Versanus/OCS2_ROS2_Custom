#include "real_robot_bridge/ContactEstimator.h"

#include <algorithm>
#include <cmath>

namespace real_robot_bridge {
namespace {

constexpr std::size_t kNumLegs = 4;
constexpr std::size_t kJointsPerLeg = 3;
constexpr std::size_t kExpectedJointCount = kNumLegs * kJointsPerLeg;

}  // namespace

ContactEstimator::ContactEstimator() : ContactEstimator(Config{}) {}

ContactEstimator::ContactEstimator(const Config& config) : config_(config) {}

std::array<bool, 4> ContactEstimator::update(const std::vector<double>& joint_torques) {
  if (joint_torques.size() < kExpectedJointCount) {
    return contact_flags_;
  }

  // Joint torques arrive in LF, LH, RF, RH order from the current MuJoCo model
  // and reference configs, while contact flags are expected as LF, RF, LH, RH.
  const std::array<std::size_t, 4> contact_to_joint_leg_map{{0, 2, 1, 3}};

  for (std::size_t contact_index = 0; contact_index < kNumLegs; ++contact_index) {
    const std::size_t leg_index = contact_to_joint_leg_map[contact_index];
    const std::size_t joint_offset = leg_index * kJointsPerLeg;

    double leg_load = 0.0;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const double torque = joint_torques[joint_offset + joint];
      leg_load += torque * torque;
    }
    leg_load = std::sqrt(leg_load);

    filtered_leg_loads_[contact_index] =
        config_.filter_alpha * filtered_leg_loads_[contact_index] + (1.0 - config_.filter_alpha) * leg_load;

    if (!contact_flags_[contact_index] && filtered_leg_loads_[contact_index] >= config_.on_threshold) {
      contact_flags_[contact_index] = true;
    } else if (contact_flags_[contact_index] && filtered_leg_loads_[contact_index] <= config_.off_threshold) {
      contact_flags_[contact_index] = false;
    }
  }

  return contact_flags_;
}

}  // namespace real_robot_bridge
