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
    double kinematic_max_height = 0.04;
    double kinematic_max_vertical_speed = 0.45;
    double kinematic_min_liftoff_vertical_speed = 0.08;
    double strong_force_margin = 1.5;
  };

  struct DebugInfo {
    double raw_normal_force = 0.0;
    double median_normal_force = 0.0;
    double filtered_normal_force = 0.0;
    double height_above_support = 0.0;
    double vertical_speed = 0.0;
    double upward_speed = 0.0;
    bool kinematic_contact_hint = false;
    bool kinematic_release_hint = false;
    bool strong_force = false;
    bool on_candidate = false;
    bool off_candidate = false;
    bool contact = false;
    bool fallback_torque_norm = false;
    int on_confirmation_count = 0;
    int off_confirmation_count = 0;
  };

  ContactEstimator();
  explicit ContactEstimator(const Config& config);

  void setConfig(const Config& config);
  void initialize(const std::string& urdf_file);
  std::array<bool, 4> update(const std::vector<double>& joint_positions,
                             const std::vector<double>& joint_velocities,
                             const std::vector<double>& joint_torques,
                             const std::vector<double>& base_pose,
                             const std::vector<double>& base_quat,
                             const std::vector<double>& base_angvel,
                             const std::vector<double>& base_linvel);
  const std::array<Eigen::Vector3d, 4>& getLastFootPositionsInBaseFrame() const { return last_foot_positions_in_base_frame_; }
  const std::array<DebugInfo, 4>& getLastDebugInfo() const { return last_debug_info_; }

 private:
  std::array<bool, 4> updateFromTorqueNorm(const std::vector<double>& joint_torques);
  double rejectOutlierAndGetMedian(std::size_t contact_index, double sample);
  void updateContactFlag(std::size_t contact_index, double filtered_normal_force,
                         bool kinematic_contact_hint, bool kinematic_release_hint,
                         bool strong_force);

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
  std::array<std::array<double, 3>, 4> raw_normal_force_history_{};
  std::array<std::size_t, 4> raw_normal_force_history_sizes_{};
  std::array<std::size_t, 4> raw_normal_force_history_indices_{};
  std::array<Eigen::Vector3d, 4> last_foot_positions_in_base_frame_{};
  std::array<DebugInfo, 4> last_debug_info_{};
};

}  // namespace real_robot_bridge
