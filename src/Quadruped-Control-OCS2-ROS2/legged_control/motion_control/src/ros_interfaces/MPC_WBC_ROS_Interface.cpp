/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "motion_control/ros_interfaces/MPC_WBC_ROS_Interface.h"
#include "motion_control/legged_interface/LeggedRobotInterface.h"
#include "motion_control/ros_interfaces/GaitReceiver.h"
#include "motion_control/ros_interfaces/RosReferenceManager.h"
#include "motion_control/legged_wbc/WeightedWbc.h"
#include "motion_control/legged_wbc/HierarchicalWbc.h"
#include "motion_control/legged_estimation/StateEstimatorFactory.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace {

bool tryLoadJointState(const std::string& referenceFile, const std::string& field, ocs2::vector_t& jointState) {
  try {
    ocs2::loadData::loadEigenMatrix(referenceFile, field, jointState);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d& position) {
  geometry_msgs::msg::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  return point;
}

std::array<float, 4> footTrajectoryColor(std::size_t index) {
  switch (index) {
    case 0:
      return {0.07f, 0.75f, 0.95f, 0.95f};
    case 1:
      return {0.95f, 0.55f, 0.15f, 0.95f};
    case 2:
      return {0.35f, 0.85f, 0.35f, 0.95f};
    case 3:
      return {0.95f, 0.25f, 0.65f, 0.95f};
    default:
      return {0.85f, 0.85f, 0.85f, 0.90f};
  }
}

}  // namespace


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_WBC_ROS_Interface::MPC_WBC_ROS_Interface(const rclcpp::Node::SharedPtr& node, 
    const std::string& taskFile,
    const std::string& urdfFile,
    const std::string& referenceFile,
    const std::string& simulatorFile,
    const std::string& robotName)
    : node_(node),
      robotName_(std::move(robotName)){

  bool verbose = false;
  ocs2::loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);
  
  //legged_interface
  leggedInterface_ = std::make_shared<LeggedRobotInterface>(taskFile, urdfFile, referenceFile);
  ocs2::loadData::loadEigenMatrix(referenceFile, "defaultJointState", standJointState_);
  sitJointState_.setZero(standJointState_.size());
  recoveryJointState_ = standJointState_;
  tryLoadJointState(referenceFile, "standJointState", standJointState_);
  tryLoadJointState(referenceFile, "sitJointState", sitJointState_);
  tryLoadJointState(referenceFile, "recoveryJointState", recoveryJointState_);
  estopHoldJointState_ = standJointState_;
  controlState_ = ControlState::Hold;
  mpcReleaseDelay_ = std::max<ocs2::scalar_t>(0.0, mpcReleaseDelay_);
  mpcBlendDuration_ = std::max<ocs2::scalar_t>(0.0, mpcBlendDuration_);
  sitDownDuration_ = std::max<ocs2::scalar_t>(0.1, sitDownDuration_);
  loadJointGainRatios(taskFile);

  // mpc
  ocs2::loadData::loadCppDataType(simulatorFile, "controller.mpc_control_frequency", Mpc_control_frequency_);
  setupMpc(robotName);
  // mpc_mrt
  setupMrt();
  // wbc
  ocs2::loadData::loadCppDataType(simulatorFile, "controller.wbc_control_frequency", Wbc_control_frequency_);
  setupWbc(taskFile, verbose);

  // State estimation
  ocs2::loadData::loadCppDataType(taskFile, "stateEstimate", StateEstimate_);
  setupStateEstimate(taskFile, verbose);
  initializeObservationBuffers();

  MpcCount_ = 0; //control the different frequencyies between mpc and wbc

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::loadJointGainRatios(const std::string& taskFile) {
  boost::property_tree::ptree tree;
  boost::property_tree::read_info(taskFile, tree);

  nominalKpRatio_ = tree.get<double>("jointGainRatios.nominalKp", nominalKpRatio_);
  nominalKdRatio_ = tree.get<double>("jointGainRatios.nominalKd", nominalKdRatio_);
  strongKpRatio_ = tree.get<double>("jointGainRatios.strongKp", strongKpRatio_);
  strongKdRatio_ = tree.get<double>("jointGainRatios.strongKd", strongKdRatio_);
  zeroKpRatio_ = tree.get<double>("jointGainRatios.zeroKp", zeroKpRatio_);
  zeroKdRatio_ = tree.get<double>("jointGainRatios.zeroKd", zeroKdRatio_);

  const auto validateRatio = [](double value, const char* name) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::runtime_error(std::string("jointGainRatios.") + name + " must be finite and non-negative.");
    }
  };
  validateRatio(nominalKpRatio_, "nominalKp");
  validateRatio(nominalKdRatio_, "nominalKd");
  validateRatio(strongKpRatio_, "strongKp");
  validateRatio(strongKdRatio_, "strongKd");
  validateRatio(zeroKpRatio_, "zeroKp");
  validateRatio(zeroKdRatio_, "zeroKd");

  RCLCPP_INFO(node_->get_logger(),
              "MPC joint gain ratios: nominal=%.3f/%.3f strong=%.3f/%.3f zero=%.3f/%.3f.",
              nominalKpRatio_, nominalKdRatio_, strongKpRatio_, strongKdRatio_, zeroKpRatio_, zeroKdRatio_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<double, double> MPC_WBC_ROS_Interface::jointGainRatiosForCurrentState(bool mpcBlendActive) const {
  if (mpcBlendActive) {
    return {strongKpRatio_, strongKdRatio_};
  }

  switch (controlState_) {
    case ControlState::Hold:
    case ControlState::RecoveryPose:
    case ControlState::SitDown:
    case ControlState::Sitting:
      return {strongKpRatio_, strongKdRatio_};
    case ControlState::ZeroTorque:
      return {zeroKpRatio_, zeroKdRatio_};
    case ControlState::Mpc:
    default:
      return {nominalKpRatio_, nominalKdRatio_};
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::setupMpc(const std::string& robotName) {
  rbdConversions_ = std::make_shared<ocs2::CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(node_, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(node_);

  mpc_ = std::make_shared<ocs2::SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::setupMrt() {
  mpcMrtInterface_ = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::setupWbc(const std::string& taskFile, bool verbose) {
  ocs2::CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                    leggedInterface_->modelSettings().contactNames3DoF);
  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::setupStateEstimate(const std::string& taskFile, bool verbose) {
  estimatorConfig_ = loadEstimatorConfig(taskFile, true);
  stateEstimate_ = StateEstimatorFactory::create(estimatorConfig_, node_, leggedInterface_->getPinocchioInterface(),
                                                 leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, taskFile, verbose);
  RCLCPP_INFO(node_->get_logger(), "Configured MPC state estimator: type=%s orientationSource=%s",
              toString(estimatorConfig_.type), toString(estimatorConfig_.orientationSource));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::initializeObservationBuffers() {
  const auto& info = leggedInterface_->getCentroidalModelInfo();
  measuredRbdState_.setZero(2 * info.generalizedCoordinatesNum);
  currentObservation_.time = 0.0;
  currentObservation_.state.setZero(info.stateDim);
  currentObservation_.input.setZero(info.inputDim);
  currentObservation_.mode = ModeNumber::STANCE;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_WBC_ROS_Interface::hasValidCurrentObservationState() const {
  return currentObservation_.state.size() == leggedInterface_->getCentroidalModelInfo().stateDim;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_WBC_ROS_Interface::shouldTrackLiveHoldPose() const {
  return controlState_ == ControlState::Hold && !mpcReleasePending_ && !mpcBlendActive_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_WBC_ROS_Interface::shouldResetAroundHoldDiscontinuity(const ocs2::vector_t& jointPos,
                                                               ocs2::scalar_t receivedTime) const {
  if (!shouldTrackLiveHoldPose()) {
    return false;
  }

  if (receivedTime + 1e-6 < currentObservation_.time) {
    return true;
  }

  const auto jointOffset = static_cast<Eigen::Index>(6);
  if (measuredRbdState_.size() < jointOffset + jointPos.size()) {
    return false;
  }

  const auto previousJointPos = measuredRbdState_.segment(jointOffset, jointPos.size());
  return (jointPos - previousJointPos).cwiseAbs().maxCoeff() > holdJointJumpThreshold_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_WBC_ROS_Interface::activePolicyExpired() const {
  if (!mpcMrtInterface_ || !mpcMrtInterface_->initialPolicyReceived()) {
    return true;
  }

  try {
    const auto& policy = mpcMrtInterface_->getPolicy();
    return policy.timeTrajectory_.empty() || currentObservation_.time >= policy.timeTrajectory_.back();
  } catch (...) {
    return true;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_WBC_ROS_Interface::recoverExpiredPolicy(const char* observationSource) {
  if (!mpcMrtInterface_ || !activePolicyExpired()) {
    return false;
  }

  if (lastMpcResetTime_ >= 0.0 && currentObservation_.time - lastMpcResetTime_ < mpcResetCooldown_) {
    return false;
  }

  try {
    const auto targetTrajectories = mpcMrtInterface_->getReferenceManager().getTargetTrajectories();
    RCLCPP_WARN(node_->get_logger(),
                "Resetting MPC/MRT after expired %s policy at t=%.3f to drop stale plan state and replan from the latest observation.",
                observationSource, currentObservation_.time);
    mpcMrtInterface_->resetMpcNode(targetTrajectories);
    mpcMrtInterface_->setCurrentObservation(currentObservation_);
    lastMpcResetTime_ = currentObservation_.time;
    MpcCount_ = 0;
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Failed to reset expired MPC policy at t=%.3f: %s",
                          currentObservation_.time, error.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Failed to reset expired MPC policy at t=%.3f due to an unknown error.",
                          currentObservation_.time);
  }

  return false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_WBC_ROS_Interface::resetMpcFromCurrentObservation(const char* reason, bool force) {
  if (!mpcMrtInterface_) {
    return false;
  }

  if (!force && lastMpcResetTime_ >= 0.0 && currentObservation_.time - lastMpcResetTime_ < mpcResetCooldown_) {
    return false;
  }

  try {
    const auto targetTrajectories = mpcMrtInterface_->getReferenceManager().getTargetTrajectories();
    RCLCPP_WARN(node_->get_logger(),
                "Resetting MPC/MRT %s at t=%.3f so the controller restarts from the current measured pose.",
                reason, currentObservation_.time);
    mpcMrtInterface_->resetMpcNode(targetTrajectories);
    mpcMrtInterface_->setCurrentObservation(currentObservation_);
    lastMpcResetTime_ = currentObservation_.time;
    MpcCount_ = 0;
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Failed to reset MPC/MRT %s at t=%.3f: %s",
                          reason, currentObservation_.time, error.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Failed to reset MPC/MRT %s at t=%.3f due to an unknown error.",
                          reason, currentObservation_.time);
  }

  return false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::resetEstimatedObservationFromCurrentSensors() {
  if (!StateEstimate_ || !stateEstimate_) {
    return;
  }

  stateEstimate_->reset();
  measuredRbdState_ = stateEstimate_->update(currentObservation_.time, 0.0);

  const ocs2::scalar_t yawLast =
      hasValidCurrentObservationState() ? currentObservation_.state(9) : ocs2::scalar_t(0.0);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) =
      yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_WBC_ROS_Interface::hasValidStateMessage(const legged_msgs::msg::SimulatorStateData& msg, std::string* reason) const {
  const auto expected_joint_count = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;

  if (msg.base_quat_values.size() < 4) {
    if (reason) *reason = "base_quat_values has fewer than 4 elements";
    return false;
  }
  if (msg.base_pose_values.size() < 3) {
    if (reason) *reason = "base_pose_values has fewer than 3 elements";
    return false;
  }
  if (msg.base_angvel_values.size() < 3) {
    if (reason) *reason = "base_angvel_values has fewer than 3 elements";
    return false;
  }
  if (msg.base_linvel_values.size() < 3) {
    if (reason) *reason = "base_linvel_values has fewer than 3 elements";
    return false;
  }
  if (msg.joint_position_values.size() < expected_joint_count) {
    if (reason) *reason = "joint_position_values is shorter than the robot actuated DoF count";
    return false;
  }
  if (msg.joint_velocity_values.size() < expected_joint_count) {
    if (reason) *reason = "joint_velocity_values is shorter than the robot actuated DoF count";
    return false;
  }

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_WBC_ROS_Interface::hasValidSensorMessage(const legged_msgs::msg::SimulatorSensorData& msg, std::string* reason) const {
  const auto expected_joint_count = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;

  if (msg.imu_quat_values.size() < 4) {
    if (reason) *reason = "imu_quat_values has fewer than 4 elements";
    return false;
  }
  if (msg.imu_angvel_values.size() < 3) {
    if (reason) *reason = "imu_angvel_values has fewer than 3 elements";
    return false;
  }
  if (msg.imu_linacc_values.size() < 3) {
    if (reason) *reason = "imu_linacc_values has fewer than 3 elements";
    return false;
  }
  if (msg.joint_position_values.size() < expected_joint_count) {
    if (reason) *reason = "joint_position_values is shorter than the robot actuated DoF count";
    return false;
  }
  if (msg.joint_velocity_values.size() < expected_joint_count) {
    if (reason) *reason = "joint_velocity_values is shorter than the robot actuated DoF count";
    return false;
  }

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::relatchHoldJointStateFromObservation() {
  if (!hasValidCurrentObservationState()) {
    return;
  }

  estopHoldJointState_ = ocs2::centroidal_model::getJointAngles(
      currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::synchronizeObservationInputWithControlState() {
  const auto inputDim = leggedInterface_->getCentroidalModelInfo().inputDim;
  if (currentObservation_.input.size() != inputDim) {
    currentObservation_.input.setZero(inputDim);
    return;
  }

  const bool controllerOwnsActuation =
      controlState_ == ControlState::Mpc && !mpcReleasePending_ && !mpcBlendActive_;
  if (!controllerOwnsActuation) {
    currentObservation_.input.setZero();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::launchNodes()
{
  RCLCPP_INFO(node_->get_logger(), "MPC node is setting up ...");

  // Observation publisher
  observationPublisher_ = node_->create_publisher<legged_msgs::msg::MpcObservation>(robotName_ + "_mpc_observation", 1);

  // Joint control publisher
  jointControlPublisher_ = node_->create_publisher<legged_msgs::msg::JointControlData>("joint_control_data", 1);
  mpcComputeTimePublisher_ = node_->create_publisher<std_msgs::msg::Float64>("mpc_compute_time_ms", 1);
  mpcFootTrajectoryPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("mpc_foot_trajectories", 1);
  emergencyOverrideStatePublisher_ = node_->create_publisher<std_msgs::msg::Int32>(
      robotName_ + "_emergency_override_state", 1);
  emergencyOverrideSubscriber_ = node_->create_subscription<std_msgs::msg::Int32>(
      robotName_ + "_emergency_override", 1,
      std::bind(&MPC_WBC_ROS_Interface::emergencyOverrideCallback, this, std::placeholders::_1));
  publishEmergencyOverrideState();

  if (StateEstimate_) // state estimate from imu data
  {
    simulatorSensorSubscriber_ = node_->create_subscription<legged_msgs::msg::SimulatorSensorData>("simulator_sensor_data", 1,
            std::bind(&MPC_WBC_ROS_Interface::simulatorSensorCallback, this, std::placeholders::_1));
  }
  else // real state data from simulator feedback
  {
    simulatorStateSubscriber_ = node_->create_subscription<legged_msgs::msg::SimulatorStateData>("simulator_state_data", 1,
      std::bind(&MPC_WBC_ROS_Interface::simulatorStateCallback, this, std::placeholders::_1));
  }

  // before spin, use service to start control
  controlStartingClient_ = node_->create_client<legged_msgs::srv::StartControl>("start_control");

  // starting control for one loop
  simulatorStartControlLoop();

  // spin
  while (rclcpp::ok()) {
    rclcpp::spin(node_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::simulatorStartControlLoop()
{
  using namespace std::chrono_literals;

  initialStateReady_ = false;
  while (rclcpp::ok()) {
    auto request = std::make_shared<legged_msgs::srv::StartControl::Request>();
    request->start = true;

    if (!controlStartingClient_->wait_for_service(2s)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Waiting for /start_control service...");
      continue;
    }

    auto result = controlStartingClient_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Timed out while waiting for /start_control response. Retrying.");
      rclcpp::sleep_for(500ms);
      continue;
    }

    auto response = result.get();
    if (!response->success) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Bridge is up but no valid robot state is available yet. Waiting for hardware feedback/IMU before starting MPC.");
      rclcpp::sleep_for(500ms);
      continue;
    }

    std::string invalid_reason;
    if (!hasValidStateMessage(response->state, &invalid_reason)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Received incomplete initial state from /start_control: %s. Retrying.",
                           invalid_reason.c_str());
      rclcpp::sleep_for(500ms);
      continue;
    }

    RCLCPP_INFO(node_->get_logger(),"starting control request success");
    RCLCPP_INFO(node_->get_logger(), "Request success: %s", response->success ? "true" : "false");

    setInitialState(std::make_shared<const legged_msgs::msg::SimulatorStateData>(response->state));
    if (!initialStateReady_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "Initial state was rejected as invalid. Retrying.");
      rclcpp::sleep_for(500ms);
      continue;
    }

    ocs2::TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
    mpcMrtInterface_->setCurrentObservation(currentObservation_);
    mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    RCLCPP_INFO(node_->get_logger(),"Waiting for the initial policy ...");
    while (!mpcMrtInterface_->initialPolicyReceived() && rclcpp::ok()) {
      mpcMrtInterface_->advanceMpc();
      rclcpp::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    }
    RCLCPP_INFO(node_->get_logger(),"Initial policy has been received.");

    mpcMrtInterface_->updatePolicy();
    publishMpcFootTrajectoryMarkers();
    // Evaluate the current policy
    ocs2::vector_t optimizedState, optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control
    currentObservation_.input = optimizedInput;
    synchronizeObservationInputWithControlState();

    wbcTimer_.startTimer();
    //RCLCPP_INFO(node_->get_logger(),"Update second: %f",1/leggedInterface_->mpcSettings().mrtDesiredFrequency_);
    ocs2::vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, 1/leggedInterface_->mpcSettings().mrtDesiredFrequency_);
    wbcTimer_.endTimer();

    ocs2::vector_t torque = x.tail(12);
    ocs2::vector_t posDes = ocs2::centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
    ocs2::vector_t velDes = ocs2::centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

    // RCLCPP_INFO(node_->get_logger(), "Torque: %s", eigenToString(torque).c_str());
    // RCLCPP_INFO(node_->get_logger(), "Position Desired: %s", eigenToString(posDes).c_str());
    // RCLCPP_INFO(node_->get_logger(), "Velocity Desired: %s", eigenToString(velDes).c_str());

    publishJointControl(torque,posDes,velDes);
    publishCurrentObservation();
    return;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string MPC_WBC_ROS_Interface::eigenToString(const ocs2::vector_t& vec) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i < vec.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::simulatorStateCallback(
    const legged_msgs::msg::SimulatorStateData::ConstSharedPtr msg) {
  //RCLCPP_INFO(node_->get_logger(), "Message recieved");
  if (!initialStateReady_) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "Ignoring simulator_state_data until initial state is ready.");
    return;
  }

  std::string invalid_reason;
  if (!hasValidStateMessage(*msg, &invalid_reason)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring invalid simulator_state_data message: %s", invalid_reason.c_str());
    return;
  }

  MpcCount_ += 1;

  updateStateEstimationFromState(msg);
  synchronizeObservationInputWithControlState();

  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  const bool forceMpcUpdate = activePolicyExpired();

  if (forceMpcUpdate) {
    recoverExpiredPolicy("state");
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Active MPC policy expired at t=%.3f. Forcing replanning from the latest state observation.",
                         currentObservation_.time);
  }

  if (forceMpcUpdate || MpcCount_ >= Wbc_control_frequency_/Mpc_control_frequency_)
  {
    MpcCount_ = 0;
    mpcTimer_.startTimer();
    mpcMrtInterface_->advanceMpc();
    mpcTimer_.endTimer();
    std::cerr << "\n###   Latest  : "
    << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]."
    << std::endl;
    publishMpcComputeTime(mpcTimer_.getLastIntervalInMilliseconds());
  }

  const bool policyUpdated = mpcMrtInterface_->updatePolicy();
  if (policyUpdated) {
    publishMpcFootTrajectoryMarkers();
  }

  if (activePolicyExpired()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Skipping stale MPC policy evaluation at t=%.3f while waiting for a refreshed plan.",
                         currentObservation_.time);
    return;
  }
// Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;
  synchronizeObservationInputWithControlState();

  wbcTimer_.startTimer();
  //RCLCPP_INFO(node_->get_logger(),"Update second: %f",1/leggedInterface_->mpcSettings().mrtDesiredFrequency_);
  ocs2::vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, 1/leggedInterface_->mpcSettings().mrtDesiredFrequency_);
  wbcTimer_.endTimer();

  ocs2::vector_t torque = x.tail(12);
  ocs2::vector_t posDes = ocs2::centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  ocs2::vector_t velDes = ocs2::centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // RCLCPP_INFO(node_->get_logger(), "Torque: %s", eigenToString(torque).c_str());
  // RCLCPP_INFO(node_->get_logger(), "Position Desired: %s", eigenToString(posDes).c_str());
  // RCLCPP_INFO(node_->get_logger(), "Velocity Desired: %s", eigenToString(velDes).c_str());

  publishJointControl(torque,posDes,velDes);
  publishCurrentObservation();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::setInitialState(
    const legged_msgs::msg::SimulatorStateData::ConstSharedPtr msg) {
  std::string invalid_reason;
  if (!hasValidStateMessage(*msg, &invalid_reason)) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring invalid initial state message: %s", invalid_reason.c_str());
    initialStateReady_ = false;
    return;
  }

  const auto expected_joint_count = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;
  // convert quat to eulerAngles
  Eigen::Quaternion<ocs2::scalar_t> quat(msg->base_quat_values[0], msg->base_quat_values[1], 
      msg->base_quat_values[2], msg->base_quat_values[3]);
  vector3_t eulerAngles = quatToZyx(quat);
  //vector3_t angularVel = {msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]};
  vector3_t angularVel(msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]);

  vector3_t position(msg->base_pose_values[0], msg->base_pose_values[1], msg->base_pose_values[2]);
  vector3_t linearVel(msg->base_linvel_values[0], msg->base_linvel_values[1], msg->base_linvel_values[2]);

  ocs2::vector_t jointPos(expected_joint_count);
  ocs2::vector_t jointVel(expected_joint_count);
  for (size_t i = 0; i < expected_joint_count; ++i) {
      jointPos(i) = msg->joint_position_values[i];
  }
  for (size_t i = 0; i < expected_joint_count; ++i) {
      jointVel(i) = msg->joint_velocity_values[i];
  }


  measuredRbdState_ = ocs2::vector_t::Zero(2 * (6 + jointPos.size()));
  measuredRbdState_.segment<3>(0) = eulerAngles;
  measuredRbdState_.segment<3>(3) = position;
  measuredRbdState_.segment(6, jointPos.size()) = jointPos;
  measuredRbdState_.segment<3>(6 + jointPos.size()) = angularVel;
  measuredRbdState_.segment<3>(9 + jointPos.size()) = linearVel;
  measuredRbdState_.segment(12 + jointPos.size(), jointVel.size()) = jointVel;

  currentObservation_.time = 0;
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  // std::cout << "measuredRbdState_: " << std::endl;
  // std::cout << measuredRbdState_ << std::endl;
  
  ocs2::scalar_t received_time = static_cast<ocs2::scalar_t>(msg->simulation_time);
  currentObservation_.time = received_time;
  ocs2::scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));

  if (controlState_ == ControlState::Hold && hasValidCurrentObservationState()) {
    relatchHoldJointStateFromObservation();
  }

  contactFlag_.fill(false);
  for (size_t i = 0; i < std::min(contactFlag_.size(), msg->contact_flags.size()); ++i) {
    contactFlag_[i] = msg->contact_flags[i];
  }

  currentObservation_.mode = stanceLeg2ModeNumber(contactFlag_);
  if (StateEstimate_ && stateEstimate_) {
    stateEstimate_->seed(measuredRbdState_);
    stateEstimate_->updateContact(contactFlag_);
  }
  initialStateReady_ = true;


}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::updateStateEstimationFromState(
    const legged_msgs::msg::SimulatorStateData::ConstSharedPtr msg) {
  std::string invalid_reason;
  if (!hasValidStateMessage(*msg, &invalid_reason)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring invalid simulator_state_data message: %s", invalid_reason.c_str());
    return;
  }

  const auto expected_joint_count = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;
  // convert quat to eulerAngles
  Eigen::Quaternion<ocs2::scalar_t> quat(msg->base_quat_values[0], msg->base_quat_values[1], 
      msg->base_quat_values[2], msg->base_quat_values[3]);
  vector3_t eulerAngles = quatToZyx(quat);
  //vector3_t angularVel = {msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]};
  vector3_t angularVel(msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]);

  vector3_t position(msg->base_pose_values[0], msg->base_pose_values[1], msg->base_pose_values[2]);
  vector3_t linearVel(msg->base_linvel_values[0], msg->base_linvel_values[1], msg->base_linvel_values[2]);

  ocs2::vector_t jointPos(expected_joint_count);
  ocs2::vector_t jointVel(expected_joint_count);
  for (size_t i = 0; i < expected_joint_count; ++i) {
      jointPos(i) = msg->joint_position_values[i];
  }
  for (size_t i = 0; i < expected_joint_count; ++i) {
      jointVel(i) = msg->joint_velocity_values[i];
  }
  const bool holdDiscontinuityDetected =
      shouldResetAroundHoldDiscontinuity(jointPos, static_cast<ocs2::scalar_t>(msg->simulation_time));
  measuredRbdState_ = ocs2::vector_t::Zero(2 * (6 + jointPos.size()));
  measuredRbdState_.segment<3>(0) = eulerAngles;
  measuredRbdState_.segment<3>(3) = position;
  measuredRbdState_.segment(6, jointPos.size()) = jointPos;
  measuredRbdState_.segment<3>(6 + jointPos.size()) = angularVel;
  measuredRbdState_.segment<3>(9 + jointPos.size()) = linearVel;
  measuredRbdState_.segment(12 + jointPos.size(), jointVel.size()) = jointVel;

  // std::cout << "measuredRbdState_: " << std::endl;
  // std::cout << measuredRbdState_ << std::endl;
  
  ocs2::scalar_t received_time = static_cast<ocs2::scalar_t>(msg->simulation_time);
  currentObservation_.time = received_time;
  ocs2::scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));

  contactFlag_.fill(false);
  for (size_t i = 0; i < std::min(contactFlag_.size(), msg->contact_flags.size()); ++i) {
    contactFlag_[i] = msg->contact_flags[i];
  }
  // for (size_t i = 0; i < contactFlag_.size(); ++i) {
  //   RCLCPP_INFO(node_->get_logger(), "Index %zu: %zu", i, static_cast<size_t>(contactFlag_[i]));
  // }
  currentObservation_.mode = stanceLeg2ModeNumber(contactFlag_);
  // RCLCPP_INFO(node_->get_logger(), "Current mode: %zu", currentObservation_.mode);

  if (holdDiscontinuityDetected) {
    relatchHoldJointStateFromObservation();
    resetMpcFromCurrentObservation("after a hold-state simulator pose jump", true);
  }

  // std::ostringstream oss;
  // oss << "SystemObservation { "
  //     << "mode: " << currentObservation_.mode << ", "
  //     << "time: " << currentObservation_.time << ", "
  //     << "state: [";

  // // print state
  // for (int i = 0; i < currentObservation_.state.size(); ++i) {
  //   oss << currentObservation_.state[i];
  //   if (i < currentObservation_.state.size() - 1) {
  //     oss << ", ";
  //   }
  // }

  // oss << "], input: [";

  // // print input 
  // for (int i = 0; i < currentObservation_.input.size(); ++i) {
  //   oss << currentObservation_.input[i];
  //   if (i < currentObservation_.input.size() - 1) {
  //     oss << ", ";
  //   }
  // }

  // oss << "] }";

  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", oss.str().c_str());

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::simulatorSensorCallback(
    const legged_msgs::msg::SimulatorSensorData::ConstSharedPtr msg) {
  //RCLCPP_INFO(node_->get_logger(), "Message recieved");
  if (!initialStateReady_) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "Ignoring simulator_sensor_data until initial state is ready.");
    return;
  }

  std::string invalid_reason;
  if (!hasValidSensorMessage(*msg, &invalid_reason)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring invalid simulator_sensor_data message: %s", invalid_reason.c_str());
    return;
  }

  MpcCount_ += 1;
  //RCLCPP_INFO(node_->get_logger(), "Message recieved");

  updateStateEstimationFromSensor(msg);
  synchronizeObservationInputWithControlState();

  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  const bool forceMpcUpdate = activePolicyExpired();

  if (forceMpcUpdate) {
    recoverExpiredPolicy("sensor");
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Active MPC policy expired at t=%.3f. Forcing replanning from the latest sensor estimate.",
                         currentObservation_.time);
  }

  if (forceMpcUpdate || MpcCount_ >= Wbc_control_frequency_/Mpc_control_frequency_)
  {
    MpcCount_ = 0;
    mpcTimer_.startTimer();
    mpcMrtInterface_->advanceMpc();
    mpcTimer_.endTimer();
    std::cerr << "\n###   Latest  : "
    << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]."
    << std::endl;
    publishMpcComputeTime(mpcTimer_.getLastIntervalInMilliseconds());
  }

  const bool policyUpdated = mpcMrtInterface_->updatePolicy();
  if (policyUpdated) {
    publishMpcFootTrajectoryMarkers();
  }

  if (activePolicyExpired()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Skipping stale MPC policy evaluation at t=%.3f while waiting for a refreshed plan.",
                         currentObservation_.time);
    return;
  }
// Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;
  synchronizeObservationInputWithControlState();

  wbcTimer_.startTimer();
  //RCLCPP_INFO(node_->get_logger(),"Update second: %f",1/leggedInterface_->mpcSettings().mrtDesiredFrequency_);
  ocs2::vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, 1/leggedInterface_->mpcSettings().mrtDesiredFrequency_);
  wbcTimer_.endTimer();

  ocs2::vector_t torque = x.tail(12);
  ocs2::vector_t posDes = ocs2::centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  ocs2::vector_t velDes = ocs2::centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // RCLCPP_INFO(node_->get_logger(), "Torque: %s", eigenToString(torque).c_str());
  // RCLCPP_INFO(node_->get_logger(), "Position Desired: %s", eigenToString(posDes).c_str());
  // RCLCPP_INFO(node_->get_logger(), "Velocity Desired: %s", eigenToString(velDes).c_str());

  publishJointControl(torque,posDes,velDes);
  publishCurrentObservation();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::updateStateEstimationFromSensor(
    const legged_msgs::msg::SimulatorSensorData::ConstSharedPtr msg) {
  std::string invalid_reason;
  if (!hasValidSensorMessage(*msg, &invalid_reason)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Ignoring invalid simulator_sensor_data message: %s", invalid_reason.c_str());
    return;
  }

  const auto expected_joint_count = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;

  Eigen::Quaternion<ocs2::scalar_t> quat(msg->imu_quat_values[0], msg->imu_quat_values[1], 
      msg->imu_quat_values[2], msg->imu_quat_values[3]);

  contactFlag_.fill(false);
  for (size_t i = 0; i < std::min(contactFlag_.size(), msg->contact_flags.size()); ++i) {
    contactFlag_[i] = msg->contact_flags[i];
  }
  //vector3_t eulerAngles = quatToZyx(quat);
  //vector3_t angularVel = {msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]};
  vector3_t angularVel(msg->imu_angvel_values[0], msg->imu_angvel_values[1], msg->imu_angvel_values[2]);
  vector3_t linearAccel(msg->imu_linacc_values[0], msg->imu_linacc_values[1], msg->imu_linacc_values[2]);

  ocs2::vector_t jointPos(expected_joint_count);
  ocs2::vector_t jointVel(expected_joint_count);
  for (size_t i = 0; i < expected_joint_count; ++i) {
      jointPos(i) = msg->joint_position_values[i];
  }
  for (size_t i = 0; i < expected_joint_count; ++i) {
      jointVel(i) = msg->joint_velocity_values[i];
  }

  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = 0;
    angularVelCovariance(i) = 0;
    linearAccelCovariance(i) = 0;
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag_);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  ocs2::scalar_t time = static_cast<ocs2::scalar_t>(msg->simulation_time);
  ocs2::scalar_t period = time - currentObservation_.time;
  const bool holdDiscontinuityDetected = shouldResetAroundHoldDiscontinuity(jointPos, time);

  if (holdDiscontinuityDetected) {
    const auto reseedState = stateEstimate_->getRbdState();
    stateEstimate_->reset();
    stateEstimate_->seed(reseedState);
    stateEstimate_->updateContact(contactFlag_);
    period = 0.0;
  }
  period = std::max<ocs2::scalar_t>(0.0, period);

  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time = time;
  ocs2::scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();

  if (holdDiscontinuityDetected) {
    relatchHoldJointStateFromObservation();
    resetMpcFromCurrentObservation("after a hold-state estimator reseed", true);
  }


  // // std::ostringstream oss;
  // // oss << "SystemObservation { "
  // //     << "mode: " << currentObservation_.mode << ", "
  // //     << "time: " << currentObservation_.time << ", "
  // //     << "state: [";

  // // // print state
  // // for (int i = 0; i < currentObservation_.state.size(); ++i) {
  // //   oss << currentObservation_.state[i];
  // //   if (i < currentObservation_.state.size() - 1) {
  // //     oss << ", ";
  // //   }
  // // }

  // // oss << "], input: [";

  // // // print input 
  // // for (int i = 0; i < currentObservation_.input.size(); ++i) {
  // //   oss << currentObservation_.input[i];
  // //   if (i < currentObservation_.input.size() - 1) {
  // //     oss << ", ";
  // //   }
  // // }

  // // oss << "] }";

  // // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", oss.str().c_str());

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::publishMpcComputeTime(double computeTimeMs) {
  if (!mpcComputeTimePublisher_) {
    return;
  }

  auto msg = std_msgs::msg::Float64();
  msg.data = computeTimeMs;
  mpcComputeTimePublisher_->publish(msg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::publishJointControl(const ocs2::vector_t& torque, const ocs2::vector_t& posDes, const ocs2::vector_t& velDes)
{
  if (mpcReleasePending_ && currentObservation_.time >= mpcReleaseTime_) {
    mpcReleasePending_ = false;
    mpcBlendActive_ = mpcBlendDuration_ > 1e-6;
    mpcBlendStartTime_ = currentObservation_.time;
    if (mpcBlendActive_) {
      RCLCPP_INFO(node_->get_logger(),
                  "Controller release delay completed. Blending back to MPC/WBC over %.2f s.",
                  mpcBlendDuration_);
    } else {
      controlState_ = ControlState::Mpc;
      RCLCPP_INFO(node_->get_logger(), "Controller release delay completed. Returning joint control to MPC/WBC.");
    }
  }

  auto msg = legged_msgs::msg::JointControlData();
  msg.joint_torque.resize(12);
  msg.joint_position.resize(12);
  msg.joint_velocity.resize(12);
  msg.kp = nominalKpRatio_;
  msg.kd = nominalKdRatio_;

  const ocs2::scalar_t blendAlpha = mpcBlendActive_
                                        ? std::clamp((currentObservation_.time - mpcBlendStartTime_) /
                                                         mpcBlendDuration_,
                                                     ocs2::scalar_t(0.0), ocs2::scalar_t(1.0))
                                        : ocs2::scalar_t(0.0);

  ocs2::vector_t rawTargetJointState = standJointState_;
  ocs2::vector_t rawTargetJointVelocity = ocs2::vector_t::Zero(standJointState_.size());

  switch (controlState_) {
    case ControlState::Hold:
      rawTargetJointState = estopHoldJointState_;
      break;
    case ControlState::RecoveryPose:
      rawTargetJointState = recoveryJointState_;
      break;
    case ControlState::SitDown: {
      const ocs2::scalar_t sitAlpha = std::clamp((currentObservation_.time - sitDownStartTime_) / sitDownDuration_,
                                                 ocs2::scalar_t(0.0), ocs2::scalar_t(1.0));
      rawTargetJointState = (1.0 - sitAlpha) * sitDownStartJointState_ + sitAlpha * sitJointState_;
      rawTargetJointVelocity = (sitJointState_ - sitDownStartJointState_) / sitDownDuration_;
      if (sitAlpha >= 1.0 - 1e-6) {
        controlState_ = ControlState::Sitting;
        rawTargetJointState = sitJointState_;
        rawTargetJointVelocity.setZero();
        RCLCPP_INFO(node_->get_logger(), "Sit-down completed. Holding sitting pose.");
      }
      break;
    }
    case ControlState::Sitting:
      rawTargetJointState = sitJointState_;
      break;
    case ControlState::ZeroTorque:
      rawTargetJointState.setZero();
      rawTargetJointVelocity.setZero();
      break;
    case ControlState::Mpc:
      break;
  }

  for (size_t i = 0; i < 12; ++i) {
    if (mpcBlendActive_) {
      msg.joint_torque[i] = blendAlpha * torque[i];
      msg.joint_position[i] = (1.0 - blendAlpha) * estopHoldJointState_[i] + blendAlpha * posDes[i];
      msg.joint_velocity[i] = blendAlpha * velDes[i];
    } else if (controlState_ == ControlState::Mpc) {
      msg.joint_torque[i] = torque[i];
      msg.joint_position[i] = posDes[i];
      msg.joint_velocity[i] = velDes[i];
    } else if (controlState_ == ControlState::ZeroTorque) {
      msg.joint_torque[i] = 0.0;
      msg.joint_position[i] = 0.0;
      msg.joint_velocity[i] = 0.0;
    } else {
      msg.joint_torque[i] = 0.0;
      msg.joint_position[i] = rawTargetJointState[i];
      msg.joint_velocity[i] = rawTargetJointVelocity[i];
    }
  }

  const auto [kpRatio, kdRatio] = jointGainRatiosForCurrentState(mpcBlendActive_);
  msg.kp = kpRatio;
  msg.kd = kdRatio;

  if (mpcBlendActive_ && blendAlpha >= 1.0 - 1e-6) {
    mpcBlendActive_ = false;
    controlState_ = ControlState::Mpc;
    RCLCPP_INFO(node_->get_logger(), "Controller blend completed. Returning joint control to MPC/WBC.");
  }

  jointControlPublisher_->publish(msg);
  publishEmergencyOverrideState();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::emergencyOverrideCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  const int command = msg->data;
  if (command == static_cast<int>(ControlCommand::Hold)) {
    mpcReleasePending_ = false;
    mpcBlendActive_ = false;
    if (hasValidCurrentObservationState()) {
      relatchHoldJointStateFromObservation();
    } else {
      estopHoldJointState_ = standJointState_;
    }
    controlState_ = ControlState::Hold;
    synchronizeObservationInputWithControlState();
    RCLCPP_WARN(node_->get_logger(), "Controller state active: HOLD current joint positions.");
  } else if (command == static_cast<int>(ControlCommand::RecoveryPose)) {
    mpcReleasePending_ = false;
    mpcBlendActive_ = false;
    controlState_ = ControlState::RecoveryPose;
    synchronizeObservationInputWithControlState();
    RCLCPP_WARN(node_->get_logger(), "Controller state active: RECOVERY pose.");
  } else if (command == static_cast<int>(ControlCommand::SitDown)) {
    if (currentObservation_.state.size() == leggedInterface_->getCentroidalModelInfo().stateDim) {
      sitDownStartJointState_ = ocs2::centroidal_model::getJointAngles(
          currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
    } else if (controlState_ == ControlState::RecoveryPose) {
      sitDownStartJointState_ = recoveryJointState_;
    } else if (controlState_ == ControlState::Hold) {
      sitDownStartJointState_ = estopHoldJointState_;
    } else {
      sitDownStartJointState_ = sitJointState_;
    }
    mpcReleasePending_ = false;
    mpcBlendActive_ = false;
    sitDownStartTime_ = currentObservation_.time;
    controlState_ = ControlState::SitDown;
    synchronizeObservationInputWithControlState();
    RCLCPP_INFO(node_->get_logger(), "Controller state active: SIT_DOWN.");
  } else if (command == static_cast<int>(ControlCommand::ZeroTorque)) {
    mpcReleasePending_ = false;
    mpcBlendActive_ = false;
    controlState_ = ControlState::ZeroTorque;
    synchronizeObservationInputWithControlState();
    RCLCPP_WARN(node_->get_logger(), "Controller state active: ZERO_TORQUE.");
  } else if (command == static_cast<int>(ControlCommand::ActivateMpc)) {
    mpcReleasePending_ = true;
    mpcBlendActive_ = false;
    mpcReleaseTime_ = currentObservation_.time + mpcReleaseDelay_;
    controlState_ = ControlState::Hold;
    resetEstimatedObservationFromCurrentSensors();
    if (hasValidCurrentObservationState()) {
      relatchHoldJointStateFromObservation();
    }
    resetMpcFromCurrentObservation("before MPC reactivation", true);
    synchronizeObservationInputWithControlState();
    RCLCPP_INFO(node_->get_logger(),
                "MPC activation requested. Holding current joints for %.2f s before returning control to MPC/WBC.",
                mpcReleaseDelay_);
  }
  publishEmergencyOverrideState();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::publishEmergencyOverrideState() {
  if (!emergencyOverrideStatePublisher_) {
    return;
  }

  auto msg = std_msgs::msg::Int32();
  msg.data = static_cast<int>(controlState_);
  emergencyOverrideStatePublisher_->publish(msg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::publishMpcFootTrajectoryMarkers() {
  if (!mpcFootTrajectoryPublisher_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markerArray;
  const auto stamp = node_->get_clock()->now();

  visualization_msgs::msg::Marker clearMarker;
  clearMarker.header.frame_id = "odom";
  clearMarker.header.stamp = stamp;
  clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
  markerArray.markers.push_back(clearMarker);

  const auto& policy = mpcMrtInterface_->getPolicy();
  if (policy.stateTrajectory_.empty()) {
    mpcFootTrajectoryPublisher_->publish(markerArray);
    return;
  }

  const auto& info = leggedInterface_->getCentroidalModelInfo();
  auto& pinocchioInterface = leggedInterface_->getPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  eeKinematicsPtr_->setPinocchioInterface(pinocchioInterface);

  std::vector<std::vector<geometry_msgs::msg::Point>> footPoints(info.numThreeDofContacts);
  for (const auto& state : policy.stateTrajectory_) {
    if (state.size() != info.stateDim) {
      continue;
    }

    pinocchio::forwardKinematics(model, data, ocs2::centroidal_model::getGeneralizedCoordinates(state, info));
    pinocchio::updateFramePlacements(model, data);

    const auto footPositions = eeKinematicsPtr_->getPosition(state);
    const auto footCount = std::min<std::size_t>(info.numThreeDofContacts, footPositions.size());
    for (std::size_t i = 0; i < footCount; ++i) {
      footPoints[i].push_back(toPointMsg(footPositions[i]));
    }
  }

  const auto& footNames = leggedInterface_->modelSettings().contactNames3DoF;
  for (std::size_t i = 0; i < footPoints.size(); ++i) {
    if (footPoints[i].empty()) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = stamp;
    marker.ns = (i < footNames.size()) ? footNames[i] : "foot_" + std::to_string(i);
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.018;
    marker.frame_locked = false;
    const auto color = footTrajectoryColor(i);
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    marker.points = std::move(footPoints[i]);
    markerArray.markers.push_back(std::move(marker));
  }

  mpcFootTrajectoryPublisher_->publish(markerArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::publishCurrentObservation()
{
  legged_msgs::msg::MpcObservation mpcObservationMsg;
  mpcObservationMsg = createObservationMsg(currentObservation_);
  observationPublisher_->publish(mpcObservationMsg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
legged_msgs::msg::MpcObservation MPC_WBC_ROS_Interface::createObservationMsg(
    const ocs2::SystemObservation& observation) {
  legged_msgs::msg::MpcObservation observationMsg;

  observationMsg.time = observation.time;

  observationMsg.state.value.resize(observation.state.rows());
  for (size_t i = 0; i < observation.state.rows(); i++) {
    observationMsg.state.value[i] = static_cast<float>(observation.state(i));
  }

  observationMsg.input.value.resize(observation.input.rows());
  for (size_t i = 0; i < observation.input.rows(); i++) {
    observationMsg.input.value[i] = static_cast<float>(observation.input(i));
  }

  observationMsg.mode = observation.mode;

  return observationMsg;
}
