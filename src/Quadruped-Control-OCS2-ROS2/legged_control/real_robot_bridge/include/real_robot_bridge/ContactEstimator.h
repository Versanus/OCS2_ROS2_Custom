#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace real_robot_bridge {

class ContactEstimator {
 public:
  struct Config {
    double filter_alpha = 0.85;
    double filter_alpha_rising = 0.85;
    double filter_alpha_falling = 0.70;
    double on_threshold = 6.0;
    double off_threshold = 3.0;
    int on_confirmation_samples = 1;
    int off_confirmation_samples = 1;
  };

  ContactEstimator();
  explicit ContactEstimator(const Config& config);

  void setConfig(const Config& config);
  void initialize(const std::string& urdf_file);
  std::array<bool, 4> update(const std::vector<double>& joint_positions,
                             const std::vector<double>& joint_velocities,
                             const std::vector<double>& joint_torques);
  const std::array<Eigen::Vector3d, 4>& getLastFootPositionsInBaseFrame() const { return last_foot_positions_in_base_frame_; }

 private:
  std::array<bool, 4> updateFromTorqueNorm(const std::vector<double>& joint_torques);

  Config config_;
  bool initialized_{false};
  std::unique_ptr<ocs2::PinocchioInterface> pinocchio_interface_ptr_;
  std::array<std::size_t, 4> contact_frame_ids_{};
  std::array<std::size_t, 12> joint_q_indices_{};
  std::array<std::size_t, 12> joint_v_indices_{};
  std::array<double, 4> filtered_normal_forces_{};
  std::array<bool, 4> contact_flags_{};
  std::array<int, 4> on_confirmation_counts_{};
  std::array<int, 4> off_confirmation_counts_{};
  std::array<Eigen::Vector3d, 4> last_foot_positions_in_base_frame_{};
};

}  // namespace real_robot_bridge
