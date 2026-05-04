#pragma once

#include <array>
#include <vector>

#include <inekf/InEKF.hpp>

#include "motion_control/legged_estimation/EstimatorConfig.h"
#include "motion_control/legged_estimation/StateEstimateBase.h"

class InvariantEkfEstimate : public StateEstimateBase {
 public:
  InvariantEkfEstimate(const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface,
                       ocs2::CentroidalModelInfo info, const ocs2::PinocchioEndEffectorKinematics& eeKinematics,
                       EstimatorConfig estimatorConfig);

  void updateImu(const Eigen::Quaternion<ocs2::scalar_t>& quat, const vector3_t& angularVelLocal,
                 const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                 const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) override;
  void updateContact(contact_flag_t contactFlag) override;
  void reset() override;
  void seed(const ocs2::vector_t& rbdState) override;
  ocs2::vector_t update(const ocs2::scalar_t& time, const ocs2::scalar_t& period) override;

  void loadSettings(const std::string& configFile, bool verbose);
  Eigen::Quaternion<ocs2::scalar_t> getOrientationQuaternion() const override;

 private:
  void initializeFilter();
  Eigen::Quaternion<ocs2::scalar_t> getBaseOrientationFromImu(
      const Eigen::Quaternion<ocs2::scalar_t>& imuOrientation) const;
  Eigen::Quaternion<ocs2::scalar_t> getImuOrientationFromBase(
      const Eigen::Quaternion<ocs2::scalar_t>& baseOrientation) const;
  vector3_t getBaseAngularVelocityLocal() const;
  vector3_t getImuLinearAccelerationForPropagation() const;
  void alignFilterRotationWithImuQuaternion();
  void applyContactHeightCorrection();
  bool initializeHeightFromContacts(vector3_t& basePosition, const Eigen::Quaternion<ocs2::scalar_t>& baseQuaternion);
  void resetContactDebounce();
  std::vector<std::pair<int, bool>> buildContactStateList() const;
  std::vector<inekf::Kinematics, Eigen::aligned_allocator<inekf::Kinematics>> buildKinematicsMeasurements();
  nav_msgs::msg::Odometry getOdomMsg(const Eigen::Quaternion<ocs2::scalar_t>& publishedQuaternion) const;

  EstimatorConfig estimatorConfig_;
  inekf::InEKF filter_;
  inekf::NoiseParams noiseParams_;
  std::vector<int> contactIds_;

  bool initialized_ = false;
  bool seeded_ = false;
  ocs2::scalar_t footRadius_ = 0.02;
  ocs2::scalar_t gyroNoise_ = 0.01;
  ocs2::scalar_t accelNoise_ = 0.10;
  ocs2::scalar_t gyroBiasNoise_ = 1e-5;
  ocs2::scalar_t accelBiasNoise_ = 1e-4;
  ocs2::scalar_t contactPositionNoise_ = 1e-4;
  ocs2::scalar_t contactVelocityNoise_ = 1e-4;
  ocs2::scalar_t contactProcessNoise_ = 0.01;
  ocs2::scalar_t jointPositionNoise_ = 1e-3;
  bool useContactVelocityCorrection_ = true;
  bool useKinematicPositionCovariance_ = true;
  bool initializeHeightFromContacts_ = true;
  int initialHeightMinContacts_ = 4;
  bool useContactHeightCorrection_ = true;
  ocs2::scalar_t contactHeight_ = 0.0;
  ocs2::scalar_t contactHeightCorrectionGain_ = 0.02;
  int contactHeightMinContacts_ = 2;
  int contactOnConfirmationSamples_ = 1;
  int contactOffConfirmationSamples_ = 1;
  bool contactDebounceInitialized_ = false;
  contact_flag_t debouncedContactFlag_{};
  std::array<int, 4> contactOnConfirmationCounts_{};
  std::array<int, 4> contactOffConfirmationCounts_{};
  bool useBaseHeightPrior_ = false;
  ocs2::scalar_t baseHeightPrior_ = 0.0;
  ocs2::scalar_t baseHeightPriorGain_ = 0.02;
  bool hasBaseHeightPrior_ = false;
  vector3_t imuPositionInBase_ = vector3_t::Zero();
  Eigen::Quaternion<ocs2::scalar_t> imuOrientationInBase_ = Eigen::Quaternion<ocs2::scalar_t>::Identity();
  bool imuLinearAccelerationIncludesGravity_ = true;
  vector3_t gravity_ = (vector3_t() << 0.0, 0.0, -9.81).finished();
  Eigen::Quaternion<ocs2::scalar_t> estimatedOrientationQuat_ = Eigen::Quaternion<ocs2::scalar_t>::Identity();
};
