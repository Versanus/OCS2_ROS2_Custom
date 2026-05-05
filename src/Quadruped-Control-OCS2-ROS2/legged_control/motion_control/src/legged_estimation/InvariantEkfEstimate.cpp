#include "motion_control/legged_estimation/InvariantEkfEstimate.h"

#include <algorithm>
#include <cmath>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

InvariantEkfEstimate::InvariantEkfEstimate(const rclcpp::Node::SharedPtr& node, ocs2::PinocchioInterface pinocchioInterface,
                                           ocs2::CentroidalModelInfo info,
                                           const ocs2::PinocchioEndEffectorKinematics& eeKinematics,
                                           EstimatorConfig estimatorConfig)
    : StateEstimateBase(node, std::move(pinocchioInterface), std::move(info), eeKinematics),
      estimatorConfig_(estimatorConfig) {
  const std::size_t numContacts = info_.numThreeDofContacts + info_.numSixDofContacts;
  contactIds_.reserve(numContacts);
  for (std::size_t i = 0; i < numContacts; ++i) {
    contactIds_.push_back(static_cast<int>(i));
  }

  eeKinematics_->setPinocchioInterface(pinocchioInterface_);
}

void InvariantEkfEstimate::updateImu(const Eigen::Quaternion<ocs2::scalar_t>& quat, const vector3_t& angularVelLocal,
                                     const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                     const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) {
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  if (estimatorConfig_.orientationSource == OrientationSource::ImuQuaternion) {
    const auto baseQuat = getBaseOrientationFromImu(quat);
    const vector3_t baseAngularVelLocal = getBaseAngularVelocityLocal();
    const vector3_t zyx = quatToZyx(baseQuat) - zyxOffset_;
    const vector3_t angularVelGlobal = ocs2::getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<ocs2::scalar_t>(
        zyx, ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(quatToZyx(baseQuat), baseAngularVelLocal));
    updateAngular(zyx, angularVelGlobal);
  }
}

void InvariantEkfEstimate::updateContact(contact_flag_t contactFlag) {
  if (!contactDebounceInitialized_) {
    debouncedContactFlag_ = contactFlag;
    contactFlag_ = debouncedContactFlag_;
    contactDebounceInitialized_ = true;
    contactOnConfirmationCounts_.fill(0);
    contactOffConfirmationCounts_.fill(0);
    return;
  }

  for (std::size_t i = 0; i < contactFlag_.size(); ++i) {
    const bool requestedContact = contactFlag[i];
    if (requestedContact == debouncedContactFlag_[i]) {
      contactOnConfirmationCounts_[i] = 0;
      contactOffConfirmationCounts_[i] = 0;
      continue;
    }

    if (requestedContact) {
      contactOffConfirmationCounts_[i] = 0;
      ++contactOnConfirmationCounts_[i];
      if (contactOnConfirmationCounts_[i] >= contactOnConfirmationSamples_) {
        debouncedContactFlag_[i] = true;
        contactOnConfirmationCounts_[i] = 0;
      }
    } else {
      contactOnConfirmationCounts_[i] = 0;
      ++contactOffConfirmationCounts_[i];
      if (contactOffConfirmationCounts_[i] >= contactOffConfirmationSamples_) {
        debouncedContactFlag_[i] = false;
        contactOffConfirmationCounts_[i] = 0;
      }
    }
  }

  contactFlag_ = debouncedContactFlag_;
}

void InvariantEkfEstimate::reset() {
  initialized_ = false;
  seeded_ = false;
  filter_ = inekf::InEKF();
  estimatedOrientationQuat_ = Eigen::Quaternion<ocs2::scalar_t>::Identity();
  resetContactDebounce();
}

void InvariantEkfEstimate::seed(const ocs2::vector_t& rbdState) {
  StateEstimateBase::seed(rbdState);
  if (useBaseHeightPrior_ && !hasBaseHeightPrior_ && rbdState_.size() >= 6) {
    baseHeightPrior_ = rbdState_(5);
    hasBaseHeightPrior_ = std::isfinite(baseHeightPrior_) && baseHeightPrior_ > 0.0;
  }
  seeded_ = true;
  initialized_ = false;
  resetContactDebounce();
}

void InvariantEkfEstimate::loadSettings(const std::string& configFile, bool verbose) {
  boost::property_tree::ptree tree;
  boost::property_tree::read_info(configFile, tree);
  const std::string prefix = "inekf.";

  footRadius_ = tree.get<ocs2::scalar_t>(prefix + "footRadius", footRadius_);
  gyroNoise_ = tree.get<ocs2::scalar_t>(prefix + "gyroNoise", gyroNoise_);
  accelNoise_ = tree.get<ocs2::scalar_t>(prefix + "accelNoise", accelNoise_);
  gyroBiasNoise_ = tree.get<ocs2::scalar_t>(prefix + "gyroBiasNoise", gyroBiasNoise_);
  accelBiasNoise_ = tree.get<ocs2::scalar_t>(prefix + "accelBiasNoise", accelBiasNoise_);
  contactPositionNoise_ = tree.get<ocs2::scalar_t>(prefix + "contactPositionNoise", contactPositionNoise_);
  contactVelocityNoise_ = tree.get<ocs2::scalar_t>(prefix + "contactVelocityNoise", contactVelocityNoise_);
  contactProcessNoise_ = tree.get<ocs2::scalar_t>(prefix + "contactProcessNoise", contactProcessNoise_);
  jointPositionNoise_ = tree.get<ocs2::scalar_t>(prefix + "jointPositionNoise", jointPositionNoise_);
  useContactVelocityCorrection_ = tree.get<bool>(prefix + "useContactVelocityCorrection", useContactVelocityCorrection_);
  useKinematicPositionCovariance_ =
      tree.get<bool>(prefix + "useKinematicPositionCovariance", useKinematicPositionCovariance_);
  initializeHeightFromContacts_ = tree.get<bool>(prefix + "initializeHeightFromContacts", initializeHeightFromContacts_);
  initialHeightMinContacts_ = tree.get<int>(prefix + "initialHeightMinContacts", initialHeightMinContacts_);
  useContactHeightCorrection_ = tree.get<bool>(prefix + "useContactHeightCorrection", useContactHeightCorrection_);
  contactHeight_ = tree.get<ocs2::scalar_t>(prefix + "contactHeight", contactHeight_);
  contactHeightCorrectionGain_ = tree.get<ocs2::scalar_t>(prefix + "contactHeightCorrectionGain", contactHeightCorrectionGain_);
  contactHeightMinContacts_ = tree.get<int>(prefix + "contactHeightMinContacts", contactHeightMinContacts_);
  contactOnConfirmationSamples_ =
      tree.get<int>(prefix + "contactOnConfirmationSamples", contactOnConfirmationSamples_);
  contactOffConfirmationSamples_ =
      tree.get<int>(prefix + "contactOffConfirmationSamples", contactOffConfirmationSamples_);
  useBaseHeightPrior_ = tree.get<bool>(prefix + "useBaseHeightPrior", useBaseHeightPrior_);
  baseHeightPrior_ = tree.get<ocs2::scalar_t>(prefix + "baseHeightPrior", baseHeightPrior_);
  baseHeightPriorGain_ = tree.get<ocs2::scalar_t>(prefix + "baseHeightPriorGain", baseHeightPriorGain_);
  imuPositionInBase_.x() = tree.get<ocs2::scalar_t>(prefix + "imuPositionInBaseX", imuPositionInBase_.x());
  imuPositionInBase_.y() = tree.get<ocs2::scalar_t>(prefix + "imuPositionInBaseY", imuPositionInBase_.y());
  imuPositionInBase_.z() = tree.get<ocs2::scalar_t>(prefix + "imuPositionInBaseZ", imuPositionInBase_.z());
  imuOrientationInBase_.w() = tree.get<ocs2::scalar_t>(prefix + "imuOrientationInBaseW", imuOrientationInBase_.w());
  imuOrientationInBase_.x() = tree.get<ocs2::scalar_t>(prefix + "imuOrientationInBaseX", imuOrientationInBase_.x());
  imuOrientationInBase_.y() = tree.get<ocs2::scalar_t>(prefix + "imuOrientationInBaseY", imuOrientationInBase_.y());
  imuOrientationInBase_.z() = tree.get<ocs2::scalar_t>(prefix + "imuOrientationInBaseZ", imuOrientationInBase_.z());
  imuLinearAccelerationIncludesGravity_ =
      tree.get<bool>(prefix + "imuLinearAccelerationIncludesGravity", imuLinearAccelerationIncludesGravity_);
  gravity_.x() = tree.get<ocs2::scalar_t>(prefix + "gravityX", gravity_.x());
  gravity_.y() = tree.get<ocs2::scalar_t>(prefix + "gravityY", gravity_.y());
  gravity_.z() = tree.get<ocs2::scalar_t>(prefix + "gravityZ", gravity_.z());
  if (imuOrientationInBase_.norm() < 1e-6) {
    imuOrientationInBase_ = Eigen::Quaternion<ocs2::scalar_t>::Identity();
  } else {
    imuOrientationInBase_.normalize();
  }
  baseHeightPriorGain_ = std::clamp(baseHeightPriorGain_, static_cast<ocs2::scalar_t>(0.0), static_cast<ocs2::scalar_t>(1.0));
  contactHeightCorrectionGain_ =
      std::clamp(contactHeightCorrectionGain_, static_cast<ocs2::scalar_t>(0.0), static_cast<ocs2::scalar_t>(1.0));
  contactHeightMinContacts_ = std::max(1, contactHeightMinContacts_);
  initialHeightMinContacts_ = std::max(1, initialHeightMinContacts_);
  contactOnConfirmationSamples_ = std::max(1, contactOnConfirmationSamples_);
  contactOffConfirmationSamples_ = std::max(1, contactOffConfirmationSamples_);
  hasBaseHeightPrior_ = useBaseHeightPrior_ && std::isfinite(baseHeightPrior_) && baseHeightPrior_ > 0.0;

  noiseParams_.setGyroscopeNoise(static_cast<double>(gyroNoise_));
  noiseParams_.setAccelerometerNoise(static_cast<double>(accelNoise_));
  noiseParams_.setGyroscopeBiasNoise(static_cast<double>(gyroBiasNoise_));
  noiseParams_.setAccelerometerBiasNoise(static_cast<double>(accelBiasNoise_));
  noiseParams_.setContactNoise(static_cast<double>(contactProcessNoise_));

  if (verbose) {
    RCLCPP_INFO(node_->get_logger(),
                "Loaded InEKF settings: orientationSource=%s footRadius=%.4f gyroNoise=%.5f accelNoise=%.5f "
                "gyroBiasNoise=%.6f accelBiasNoise=%.6f contactPositionNoise=%.6f contactVelocityNoise=%.6f "
                "contactProcessNoise=%.6f jointPositionNoise=%.6f useContactVelocityCorrection=%s "
                "useKinematicPositionCovariance=%s initializeHeightFromContacts=%s initialHeightMinContacts=%d "
                "useContactHeightCorrection=%s "
                "contactHeight=%.4f contactHeightCorrectionGain=%.3f contactHeightMinContacts=%d "
                "contactOnConfirmationSamples=%d contactOffConfirmationSamples=%d useBaseHeightPrior=%s "
                "baseHeightPrior=%.4f baseHeightPriorGain=%.3f "
                "imuPositionInBase=[%.4f, %.4f, %.4f] imuOrientationInBase=[%.4f, %.4f, %.4f, %.4f] "
                "imuLinearAccelerationIncludesGravity=%s gravity=[%.3f, %.3f, %.3f]",
                toString(estimatorConfig_.orientationSource), footRadius_, gyroNoise_, accelNoise_, gyroBiasNoise_,
                accelBiasNoise_, contactPositionNoise_, contactVelocityNoise_,
                contactProcessNoise_, jointPositionNoise_,
                useContactVelocityCorrection_ ? "true" : "false",
                useKinematicPositionCovariance_ ? "true" : "false",
                initializeHeightFromContacts_ ? "true" : "false",
                initialHeightMinContacts_,
                useContactHeightCorrection_ ? "true" : "false",
                contactHeight_, contactHeightCorrectionGain_, contactHeightMinContacts_,
                contactOnConfirmationSamples_, contactOffConfirmationSamples_,
                useBaseHeightPrior_ ? "true" : "false", baseHeightPrior_, baseHeightPriorGain_,
                imuPositionInBase_.x(), imuPositionInBase_.y(), imuPositionInBase_.z(),
                imuOrientationInBase_.w(), imuOrientationInBase_.x(), imuOrientationInBase_.y(), imuOrientationInBase_.z(),
                imuLinearAccelerationIncludesGravity_ ? "true" : "false",
                gravity_.x(), gravity_.y(), gravity_.z());
  }
}

Eigen::Quaternion<ocs2::scalar_t> InvariantEkfEstimate::getOrientationQuaternion() const {
  const auto imuOrientation =
      estimatorConfig_.orientationSource == OrientationSource::Ekf ? estimatedOrientationQuat_ : quat_;
  return getBaseOrientationFromImu(imuOrientation);
}

Eigen::Quaternion<ocs2::scalar_t> InvariantEkfEstimate::getBaseOrientationFromImu(
    const Eigen::Quaternion<ocs2::scalar_t>& imuOrientation) const {
  Eigen::Quaternion<ocs2::scalar_t> baseOrientation = imuOrientation * imuOrientationInBase_.conjugate();
  baseOrientation.normalize();
  return baseOrientation;
}

Eigen::Quaternion<ocs2::scalar_t> InvariantEkfEstimate::getImuOrientationFromBase(
    const Eigen::Quaternion<ocs2::scalar_t>& baseOrientation) const {
  Eigen::Quaternion<ocs2::scalar_t> imuOrientation = baseOrientation * imuOrientationInBase_;
  imuOrientation.normalize();
  return imuOrientation;
}

vector3_t InvariantEkfEstimate::getBaseAngularVelocityLocal() const {
  return imuOrientationInBase_.toRotationMatrix() * angularVelLocal_;
}

vector3_t InvariantEkfEstimate::getImuLinearAccelerationForPropagation() const {
  if (imuLinearAccelerationIncludesGravity_) {
    return linearAccelLocal_;
  }

  const matrix3_t rotationWorldFromImu = filter_.getState().getRotation().cast<ocs2::scalar_t>();
  return linearAccelLocal_ - rotationWorldFromImu.transpose() * gravity_;
}

void InvariantEkfEstimate::alignFilterRotationWithImuQuaternion() {
  if (!initialized_ || estimatorConfig_.orientationSource != OrientationSource::ImuQuaternion || quat_.norm() < 1e-6) {
    return;
  }

  Eigen::Quaternion<ocs2::scalar_t> imuOrientation = quat_;
  imuOrientation.normalize();

  auto state = filter_.getState();
  state.setRotation(imuOrientation.toRotationMatrix().cast<double>());
  filter_.setState(state);
  estimatedOrientationQuat_ = imuOrientation;
}

void InvariantEkfEstimate::applyContactHeightCorrection() {
  if (!useContactHeightCorrection_ || contactHeightCorrectionGain_ <= 0.0) {
    return;
  }

  const auto estimatedContacts = filter_.getEstimatedContactPositions();
  if (estimatedContacts.empty()) {
    return;
  }

  auto state = filter_.getState();
  Eigen::MatrixXd stateMatrix = state.getX();
  ocs2::scalar_t activeContactHeightSum = 0.0;
  int activeContacts = 0;

  for (std::size_t i = 0; i < contactIds_.size(); ++i) {
    if (i >= contactFlag_.size() || !contactFlag_[i]) {
      continue;
    }
    const auto contactIt = estimatedContacts.find(contactIds_[i]);
    if (contactIt == estimatedContacts.end() || contactIt->second < 0 || contactIt->second >= stateMatrix.cols()) {
      continue;
    }

    activeContactHeightSum += static_cast<ocs2::scalar_t>(stateMatrix(2, contactIt->second));
    ++activeContacts;
  }

  if (activeContacts < contactHeightMinContacts_) {
    return;
  }

  const ocs2::scalar_t meanContactHeight = activeContactHeightSum / static_cast<ocs2::scalar_t>(activeContacts);
  const ocs2::scalar_t targetFootFrameHeight = contactHeight_ + footRadius_;
  const ocs2::scalar_t heightDelta = contactHeightCorrectionGain_ * (targetFootFrameHeight - meanContactHeight);
  if (!std::isfinite(heightDelta)) {
    return;
  }

  stateMatrix(2, 4) += static_cast<double>(heightDelta);
  for (const auto& contact : estimatedContacts) {
    if (contact.second >= 0 && contact.second < stateMatrix.cols()) {
      stateMatrix(2, contact.second) += static_cast<double>(heightDelta);
    }
  }

  state.setX(stateMatrix);
  filter_.setState(state);
}

bool InvariantEkfEstimate::initializeHeightFromContacts(vector3_t& basePosition,
                                                        const Eigen::Quaternion<ocs2::scalar_t>& baseQuaternion) {
  if (!initializeHeightFromContacts_ || rbdState_.size() < static_cast<Eigen::Index>(6 + info_.actuatedDofNum)) {
    return false;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const std::size_t actuatedDofNum = info_.actuatedDofNum;

  ocs2::vector_t qPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.segment<3>(3) = quatToZyx(baseQuaternion);
  qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

  pinocchio::forwardKinematics(model, data, qPino);
  pinocchio::updateFramePlacements(model, data);

  const auto eePositions = eeKinematics_->getPosition(ocs2::vector_t());
  ocs2::scalar_t activeFootHeightSum = 0.0;
  int activeContacts = 0;
  for (std::size_t i = 0; i < contactIds_.size() && i < eePositions.size(); ++i) {
    if (i >= contactFlag_.size() || !contactFlag_[i]) {
      continue;
    }
    activeFootHeightSum += eePositions[i].z();
    ++activeContacts;
  }

  if (activeContacts < initialHeightMinContacts_) {
    return false;
  }

  const ocs2::scalar_t averageFootHeight = activeFootHeightSum / static_cast<ocs2::scalar_t>(activeContacts);
  basePosition.z() = contactHeight_ - averageFootHeight + footRadius_;
  return true;
}

void InvariantEkfEstimate::resetContactDebounce() {
  contactDebounceInitialized_ = false;
  debouncedContactFlag_.fill(false);
  contactOnConfirmationCounts_.fill(0);
  contactOffConfirmationCounts_.fill(0);
}

void InvariantEkfEstimate::initializeFilter() {
  inekf::RobotState initialState;

  Eigen::Quaternion<ocs2::scalar_t> initialBaseQuaternion = Eigen::Quaternion<ocs2::scalar_t>::Identity();
  if (quat_.norm() >= 1e-6) {
    initialBaseQuaternion = getBaseOrientationFromImu(quat_);
  }
  vector3_t initialBasePosition = vector3_t::Zero();
  vector3_t initialBaseLinearVelocity = vector3_t::Zero();
  vector3_t initialBaseAngularVelocity = vector3_t::Zero();
  if (seeded_ && rbdState_.size() >= 12) {
    initialBaseQuaternion = ocs2::getQuaternionFromEulerAnglesZyx(vector3_t(rbdState_.segment<3>(0)));
    initialBasePosition = rbdState_.segment<3>(3);
    initialBaseLinearVelocity = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
    initialBaseAngularVelocity = rbdState_.segment<3>(info_.generalizedCoordinatesNum);
  }
  if (initialBaseQuaternion.norm() < 1e-6) {
    initialBaseQuaternion = Eigen::Quaternion<ocs2::scalar_t>::Identity();
  }
  initialBaseQuaternion.normalize();
  initializeHeightFromContacts(initialBasePosition, initialBaseQuaternion);

  const auto baseRotation = initialBaseQuaternion.toRotationMatrix();
  Eigen::Quaternion<ocs2::scalar_t> initialQuaternion = getImuOrientationFromBase(initialBaseQuaternion);
  const vector3_t initialPosition = initialBasePosition + baseRotation * imuPositionInBase_;
  const vector3_t initialLinearVelocity =
      initialBaseLinearVelocity + baseRotation * initialBaseAngularVelocity.cross(imuPositionInBase_);

  initialState.setRotation(initialQuaternion.toRotationMatrix().cast<double>());
  initialState.setVelocity(initialLinearVelocity.cast<double>());
  initialState.setPosition(initialPosition.cast<double>());
  initialState.setGyroscopeBias(Eigen::Vector3d::Zero());
  initialState.setAccelerometerBias(Eigen::Vector3d::Zero());

  filter_ = inekf::InEKF(initialState, noiseParams_);
  filter_.setGravity(gravity_.cast<double>());
  filter_.setContacts(buildContactStateList());
  estimatedOrientationQuat_ = initialQuaternion;
  initialized_ = true;
}

std::vector<std::pair<int, bool>> InvariantEkfEstimate::buildContactStateList() const {
  std::vector<std::pair<int, bool>> contacts;
  contacts.reserve(contactIds_.size());
  for (std::size_t i = 0; i < contactIds_.size(); ++i) {
    contacts.emplace_back(contactIds_[i], i < contactFlag_.size() ? contactFlag_[i] : false);
  }
  return contacts;
}

std::vector<inekf::Kinematics, Eigen::aligned_allocator<inekf::Kinematics>> InvariantEkfEstimate::buildKinematicsMeasurements() {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const std::size_t actuatedDofNum = info_.actuatedDofNum;

  const Eigen::Matrix3d rotationImuFromBase = imuOrientationInBase_.toRotationMatrix().transpose().cast<double>();

  ocs2::vector_t qPino(info_.generalizedCoordinatesNum);
  ocs2::vector_t vPino(info_.generalizedCoordinatesNum);
  qPino.setZero();
  qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

  vPino.setZero();
  vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

  pinocchio::forwardKinematics(model, data, qPino, vPino);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, qPino);

  const auto eePositions = eeKinematics_->getPosition(ocs2::vector_t());
  const Eigen::Matrix3d velocityCovariance = contactVelocityNoise_ * Eigen::Matrix3d::Identity();

  std::vector<inekf::Kinematics, Eigen::aligned_allocator<inekf::Kinematics>> measurements;
  measurements.reserve(contactIds_.size());
  for (std::size_t i = 0; i < contactIds_.size(); ++i) {
    const Eigen::Vector3d footPositionBase = eePositions[i].cast<double>();
    const Eigen::Vector3d position = rotationImuFromBase * (footPositionBase - imuPositionInBase_.cast<double>());
    Eigen::Matrix3d positionCovariance = contactPositionNoise_ * Eigen::Matrix3d::Identity();
    if (useKinematicPositionCovariance_ && i < info_.endEffectorFrameIndices.size()) {
      Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, model.nv);
      pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL, jacobian);
      const Eigen::MatrixXd jointJacobian = jacobian.block(0, 6, 3, static_cast<Eigen::Index>(actuatedDofNum));
      positionCovariance += static_cast<double>(jointPositionNoise_) * jointJacobian * jointJacobian.transpose();
    }
    if (useContactVelocityCorrection_) {
      const Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
      measurements.emplace_back(contactIds_[i], position, positionCovariance, velocity, velocityCovariance);
    } else {
      measurements.emplace_back(contactIds_[i], position, positionCovariance);
    }
  }
  return measurements;
}

nav_msgs::msg::Odometry InvariantEkfEstimate::getOdomMsg(const Eigen::Quaternion<ocs2::scalar_t>& publishedQuaternion) const {
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = rbdState_.segment<3>(3).x();
  odom.pose.pose.position.y = rbdState_.segment<3>(3).y();
  odom.pose.pose.position.z = rbdState_.segment<3>(3).z();
  odom.pose.pose.orientation.w = publishedQuaternion.w();
  odom.pose.pose.orientation.x = publishedQuaternion.x();
  odom.pose.pose.orientation.y = publishedQuaternion.y();
  odom.pose.pose.orientation.z = publishedQuaternion.z();

  const Eigen::Matrix3d rotation = publishedQuaternion.toRotationMatrix();
  const vector3_t twistLinear = rotation.transpose() * rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
  odom.twist.twist.linear.x = twistLinear.x();
  odom.twist.twist.linear.y = twistLinear.y();
  odom.twist.twist.linear.z = twistLinear.z();
  const vector3_t baseAngularVelLocal = getBaseAngularVelocityLocal();
  odom.twist.twist.angular.x = baseAngularVelLocal.x();
  odom.twist.twist.angular.y = baseAngularVelLocal.y();
  odom.twist.twist.angular.z = baseAngularVelLocal.z();
  return odom;
}

ocs2::vector_t InvariantEkfEstimate::update(const ocs2::scalar_t& time, const ocs2::scalar_t& period) {
  if (!initialized_) {
    initializeFilter();
  }

  alignFilterRotationWithImuQuaternion();

  if (period > 0.0) {
    Eigen::Matrix<double, 6, 1> imuMeasurement;
    imuMeasurement.head<3>() = angularVelLocal_.cast<double>();
    imuMeasurement.tail<3>() = getImuLinearAccelerationForPropagation().cast<double>();
    filter_.propagate(imuMeasurement, static_cast<double>(period));
  }

  alignFilterRotationWithImuQuaternion();

  filter_.setContacts(buildContactStateList());
  const auto kinematicsMeasurements = buildKinematicsMeasurements();
  filter_.correctKinematics(kinematicsMeasurements);
  alignFilterRotationWithImuQuaternion();
  applyContactHeightCorrection();

  const auto& state = filter_.getState();
  estimatedOrientationQuat_ = Eigen::Quaternion<ocs2::scalar_t>(state.getRotation().cast<ocs2::scalar_t>());
  estimatedOrientationQuat_.normalize();
  const Eigen::Quaternion<ocs2::scalar_t> publishedQuaternion =
      estimatorConfig_.orientationSource == OrientationSource::Ekf ? getBaseOrientationFromImu(estimatedOrientationQuat_)
                                                                   : getBaseOrientationFromImu(quat_);
  const Eigen::Matrix3d rotationWorldFromBase = publishedQuaternion.toRotationMatrix().cast<double>();
  vector3_t estimatedPosition =
      state.getPosition().cast<ocs2::scalar_t>() - rotationWorldFromBase.cast<ocs2::scalar_t>() * imuPositionInBase_;
  vector3_t estimatedLinearVelocity =
      state.getVelocity().cast<ocs2::scalar_t>() - rotationWorldFromBase.cast<ocs2::scalar_t>() *
                                                       getBaseAngularVelocityLocal().cross(imuPositionInBase_);

  if (useBaseHeightPrior_ && hasBaseHeightPrior_) {
    const ocs2::scalar_t heightError = baseHeightPrior_ - estimatedPosition.z();
    estimatedPosition.z() += baseHeightPriorGain_ * heightError;

    auto correctedState = state;
    const vector3_t correctedImuPosition =
        estimatedPosition + rotationWorldFromBase.cast<ocs2::scalar_t>() * imuPositionInBase_;
    correctedState.setPosition(correctedImuPosition.cast<double>());
    filter_.setState(correctedState);
  }

  const vector3_t publishedEulerAngles = quatToZyx(publishedQuaternion) - zyxOffset_;
  const vector3_t baseAngularVelLocal = getBaseAngularVelocityLocal();
  const vector3_t angularVelGlobal = ocs2::getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<ocs2::scalar_t>(
      publishedEulerAngles,
      ocs2::getEulerAnglesZyxDerivativesFromLocalAngularVelocity<ocs2::scalar_t>(quatToZyx(publishedQuaternion), baseAngularVelLocal));

  updateAngular(publishedEulerAngles, angularVelGlobal);
  updateLinear(estimatedPosition, estimatedLinearVelocity);

  auto odom = getOdomMsg(publishedQuaternion);
  odom.header.stamp = rclcpp::Time(time);
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base";
  publishMsgs(odom);

  return rbdState_;
}
