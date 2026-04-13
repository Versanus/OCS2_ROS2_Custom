#include "real_robot_bridge/ContactEstimator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace real_robot_bridge {
namespace {

constexpr std::size_t kNumLegs = 4;
constexpr std::size_t kJointsPerLeg = 3;
constexpr std::size_t kExpectedJointCount = kNumLegs * kJointsPerLeg;

const std::array<std::size_t, 4> kContactToInputLegMap{{0, 2, 1, 3}};

const std::array<const char*, kExpectedJointCount> kInputJointNames{{
    "LF_HAA", "LF_HFE", "LF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
}};

const std::array<const char*, kNumLegs> kContactFrameNames{{
    "LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT",
}};

double applyAsymmetricLowPass(double previous_value, double measured_value,
                              double alpha_rising, double alpha_falling) {
  const double alpha = (measured_value >= previous_value) ? alpha_rising : alpha_falling;
  const double clamped_alpha = std::clamp(alpha, 0.0, 0.999);
  return clamped_alpha * previous_value + (1.0 - clamped_alpha) * measured_value;
}

}  // namespace

ContactEstimator::ContactEstimator() : ContactEstimator(Config{}) {}

ContactEstimator::ContactEstimator(const Config& config) : config_(config) {}

void ContactEstimator::setConfig(const Config& config) {
  config_ = config;
  config_.filter_alpha_rising = std::clamp(config_.filter_alpha_rising, 0.0, 0.999);
  config_.filter_alpha_falling = std::clamp(config_.filter_alpha_falling, 0.0, 0.999);
  config_.on_confirmation_samples = std::max(1, config_.on_confirmation_samples);
  config_.off_confirmation_samples = std::max(1, config_.off_confirmation_samples);
  filtered_normal_forces_.fill(0.0);
  contact_flags_.fill(false);
  on_confirmation_counts_.fill(0);
  off_confirmation_counts_.fill(0);
}

void ContactEstimator::initialize(const std::string& urdf_file) {
  if (urdf_file.empty()) {
    throw std::runtime_error("ContactEstimator requires a non-empty URDF path.");
  }

  const std::vector<std::string> joint_names(kInputJointNames.begin(), kInputJointNames.end());
  pinocchio_interface_ptr_ = std::make_unique<ocs2::PinocchioInterface>(
      ocs2::centroidal_model::createPinocchioInterface(urdf_file, joint_names));

  const auto& model = pinocchio_interface_ptr_->getModel();
  for (std::size_t i = 0; i < kExpectedJointCount; ++i) {
    const auto joint_id = model.getJointId(kInputJointNames[i]);
    if (joint_id == 0) {
      throw std::runtime_error(std::string("ContactEstimator could not find joint '") + kInputJointNames[i] + "' in URDF.");
    }

    joint_q_indices_[i] = model.joints[joint_id].idx_q();
    joint_v_indices_[i] = model.joints[joint_id].idx_v();
  }

  for (std::size_t i = 0; i < kNumLegs; ++i) {
    const auto frame_id = model.getBodyId(kContactFrameNames[i]);
    if (frame_id == 0) {
      throw std::runtime_error(std::string("ContactEstimator could not find contact body '") + kContactFrameNames[i] + "' in URDF.");
    }
    contact_frame_ids_[i] = frame_id;
  }

  initialized_ = true;
}

std::array<bool, 4> ContactEstimator::update(const std::vector<double>& joint_positions,
                                             const std::vector<double>& joint_velocities,
                                             const std::vector<double>& joint_torques) {
  if (joint_positions.size() < kExpectedJointCount || joint_velocities.size() < kExpectedJointCount ||
      joint_torques.size() < kExpectedJointCount || !initialized_ || !pinocchio_interface_ptr_) {
    return updateFromTorqueNorm(joint_torques);
  }

  const auto& model = pinocchio_interface_ptr_->getModel();
  auto& data = pinocchio_interface_ptr_->getData();

  auto q = pinocchio::neutral(model);
  ocs2::vector_t v = ocs2::vector_t::Zero(model.nv);

  for (std::size_t i = 0; i < kExpectedJointCount; ++i) {
    q(joint_q_indices_[i]) = joint_positions[i];
    v(joint_v_indices_[i]) = joint_velocities[i];
  }

  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  const ocs2::vector_t dynamic_bias = pinocchio::nonLinearEffects(model, data, q, v);

  for (std::size_t contact_index = 0; contact_index < kNumLegs; ++contact_index) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_world = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv);
    pinocchio::getFrameJacobian(model, data, contact_frame_ids_[contact_index], pinocchio::LOCAL_WORLD_ALIGNED, jacobian_world);
    last_foot_positions_in_base_frame_[contact_index] = data.oMf[contact_frame_ids_[contact_index]].translation();

    const std::size_t input_leg_index = kContactToInputLegMap[contact_index];
    Eigen::Matrix3d leg_jacobian_transpose = Eigen::Matrix3d::Zero();
    Eigen::Vector3d leg_torque = Eigen::Vector3d::Zero();
    Eigen::Vector3d leg_bias = Eigen::Vector3d::Zero();

    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::size_t input_joint_index = input_leg_index * kJointsPerLeg + joint;
      const auto velocity_index = static_cast<Eigen::Index>(joint_v_indices_[input_joint_index]);
      leg_jacobian_transpose.row(joint) = jacobian_world.topRows<3>().col(velocity_index).transpose();
      leg_torque(joint) = joint_torques[input_joint_index];
      leg_bias(joint) = dynamic_bias(velocity_index);
    }

    const Eigen::Vector3d estimated_force =
        -leg_jacobian_transpose.completeOrthogonalDecomposition().solve(leg_torque - leg_bias);
    const double normal_force = std::abs(estimated_force.z());

    filtered_normal_forces_[contact_index] = applyAsymmetricLowPass(
        filtered_normal_forces_[contact_index], normal_force,
        config_.filter_alpha_rising, config_.filter_alpha_falling);

    if (!contact_flags_[contact_index]) {
      off_confirmation_counts_[contact_index] = 0;
      if (filtered_normal_forces_[contact_index] >= config_.on_threshold) {
        ++on_confirmation_counts_[contact_index];
        if (on_confirmation_counts_[contact_index] >= config_.on_confirmation_samples) {
          contact_flags_[contact_index] = true;
          on_confirmation_counts_[contact_index] = 0;
        }
      } else {
        on_confirmation_counts_[contact_index] = 0;
      }
    } else {
      on_confirmation_counts_[contact_index] = 0;
      if (filtered_normal_forces_[contact_index] <= config_.off_threshold) {
        ++off_confirmation_counts_[contact_index];
        if (off_confirmation_counts_[contact_index] >= config_.off_confirmation_samples) {
          contact_flags_[contact_index] = false;
          off_confirmation_counts_[contact_index] = 0;
        }
      } else {
        off_confirmation_counts_[contact_index] = 0;
      }
    }
  }

  return contact_flags_;
}

std::array<bool, 4> ContactEstimator::updateFromTorqueNorm(const std::vector<double>& joint_torques) {
  if (joint_torques.size() < kExpectedJointCount) {
    return contact_flags_;
  }

  for (std::size_t contact_index = 0; contact_index < kNumLegs; ++contact_index) {
    const std::size_t input_leg_index = kContactToInputLegMap[contact_index];
    const std::size_t joint_offset = input_leg_index * kJointsPerLeg;

    double leg_load = 0.0;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const double torque = joint_torques[joint_offset + joint];
      leg_load += torque * torque;
    }
    leg_load = std::sqrt(leg_load);

    filtered_normal_forces_[contact_index] = applyAsymmetricLowPass(
        filtered_normal_forces_[contact_index], leg_load,
        config_.filter_alpha_rising, config_.filter_alpha_falling);

    if (!contact_flags_[contact_index]) {
      off_confirmation_counts_[contact_index] = 0;
      if (filtered_normal_forces_[contact_index] >= config_.on_threshold) {
        ++on_confirmation_counts_[contact_index];
        if (on_confirmation_counts_[contact_index] >= config_.on_confirmation_samples) {
          contact_flags_[contact_index] = true;
          on_confirmation_counts_[contact_index] = 0;
        }
      } else {
        on_confirmation_counts_[contact_index] = 0;
      }
    } else {
      on_confirmation_counts_[contact_index] = 0;
      if (filtered_normal_forces_[contact_index] <= config_.off_threshold) {
        ++off_confirmation_counts_[contact_index];
        if (off_confirmation_counts_[contact_index] >= config_.off_confirmation_samples) {
          contact_flags_[contact_index] = false;
          off_confirmation_counts_[contact_index] = 0;
        }
      } else {
        off_confirmation_counts_[contact_index] = 0;
      }
    }
  }

  return contact_flags_;
}

}  // namespace real_robot_bridge
