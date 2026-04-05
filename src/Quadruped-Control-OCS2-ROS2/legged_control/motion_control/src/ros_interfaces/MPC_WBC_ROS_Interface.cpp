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
#include "motion_control/legged_estimation/LinearKalmanFilter.h"

#include <algorithm>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>

namespace {

bool tryLoadJointState(const std::string& referenceFile, const std::string& field, ocs2::vector_t& jointState) {
  try {
    ocs2::loadData::loadEigenMatrix(referenceFile, field, jointState);
    return true;
  } catch (const std::exception&) {
    return false;
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

  MpcCount_ = 0; //control the different frequencyies between mpc and wbc

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
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(node_, leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  //currentObservation_.time = 0;
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
  auto request = std::make_shared<legged_msgs::srv::StartControl::Request>();
  request->start = true;

  if (!controlStartingClient_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_WARN(node_->get_logger(), "Service not available.");
      return;
  }

  auto result = controlStartingClient_->async_send_request(request);                                        

  if (rclcpp::spin_until_future_complete(node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS)                                                    
  {
    auto response = result.get();
    RCLCPP_INFO(node_->get_logger(),"starting control request success");
    RCLCPP_INFO(node_->get_logger(), "Request success: %s", response->success ? "true" : "false");
    auto state = response->state;
    
    setInitialState(std::make_shared<const legged_msgs::msg::SimulatorStateData>(state));
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
    // Evaluate the current policy
    ocs2::vector_t optimizedState, optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control
    currentObservation_.input = optimizedInput;

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
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Starting control service failed!!!");
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
  MpcCount_ += 1;

  updateStateEstimationFromState(msg);

  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  if (MpcCount_ >= Wbc_control_frequency_/Mpc_control_frequency_)
  {
    MpcCount_ = 0;
    mpcTimer_.startTimer();
    mpcMrtInterface_->advanceMpc();
    mpcTimer_.endTimer();
    std::cerr << "\n###   Latest  : "
    << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]."
    << std::endl;
  }

  mpcMrtInterface_->updatePolicy();
// Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;

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
  // convert quat to eulerAngles
  Eigen::Quaternion<ocs2::scalar_t> quat(msg->base_quat_values[0], msg->base_quat_values[1], 
      msg->base_quat_values[2], msg->base_quat_values[3]);
  vector3_t eulerAngles = quatToZyx(quat);
  //vector3_t angularVel = {msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]};
  vector3_t angularVel(msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]);

  vector3_t position(msg->base_pose_values[0], msg->base_pose_values[1], msg->base_pose_values[2]);
  vector3_t linearVel(msg->base_linvel_values[0], msg->base_linvel_values[1], msg->base_linvel_values[2]);

  ocs2::vector_t jointPos(msg->joint_position_values.size());
  ocs2::vector_t jointVel(msg->joint_velocity_values.size());
  for (size_t i = 0; i < msg->joint_position_values.size(); ++i) {
      jointPos(i) = msg->joint_position_values[i];
  }
  for (size_t i = 0; i < msg->joint_velocity_values.size(); ++i) {
      jointVel(i) = msg->joint_velocity_values[i];
  }


  measuredRbdState_ = ocs2::vector_t::Zero(2 * (6 + jointPos.size()));
  measuredRbdState_.segment(6, jointPos.size()) = jointPos;
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

  if (controlState_ == ControlState::Hold &&
      currentObservation_.state.size() == leggedInterface_->getCentroidalModelInfo().stateDim) {
    estopHoldJointState_ = ocs2::centroidal_model::getJointAngles(
        currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
  }

  for (size_t i = 0; i < msg->contact_flags.size(); ++i) {
    contactFlag_[i] = msg->contact_flags[i];
  }

  currentObservation_.mode = stanceLeg2ModeNumber(contactFlag_);


}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_WBC_ROS_Interface::updateStateEstimationFromState(
    const legged_msgs::msg::SimulatorStateData::ConstSharedPtr msg) {
  // convert quat to eulerAngles
  Eigen::Quaternion<ocs2::scalar_t> quat(msg->base_quat_values[0], msg->base_quat_values[1], 
      msg->base_quat_values[2], msg->base_quat_values[3]);
  vector3_t eulerAngles = quatToZyx(quat);
  //vector3_t angularVel = {msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]};
  vector3_t angularVel(msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]);

  vector3_t position(msg->base_pose_values[0], msg->base_pose_values[1], msg->base_pose_values[2]);
  vector3_t linearVel(msg->base_linvel_values[0], msg->base_linvel_values[1], msg->base_linvel_values[2]);

  ocs2::vector_t jointPos(msg->joint_position_values.size());
  ocs2::vector_t jointVel(msg->joint_velocity_values.size());
  for (size_t i = 0; i < msg->joint_position_values.size(); ++i) {
      jointPos(i) = msg->joint_position_values[i];
  }
  for (size_t i = 0; i < msg->joint_velocity_values.size(); ++i) {
      jointVel(i) = msg->joint_velocity_values[i];
  }
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

  for (size_t i = 0; i < msg->contact_flags.size(); ++i) {
    contactFlag_[i] = msg->contact_flags[i];
  }
  // for (size_t i = 0; i < contactFlag_.size(); ++i) {
  //   RCLCPP_INFO(node_->get_logger(), "Index %zu: %zu", i, static_cast<size_t>(contactFlag_[i]));
  // }
  currentObservation_.mode = stanceLeg2ModeNumber(contactFlag_);
  // RCLCPP_INFO(node_->get_logger(), "Current mode: %zu", currentObservation_.mode);

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
  MpcCount_ += 1;
  //RCLCPP_INFO(node_->get_logger(), "Message recieved");

  updateStateEstimationFromSensor(msg);

  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  if (MpcCount_ >= Wbc_control_frequency_/Mpc_control_frequency_)
  {
    MpcCount_ = 0;
    mpcTimer_.startTimer();
    mpcMrtInterface_->advanceMpc();
    mpcTimer_.endTimer();
    std::cerr << "\n###   Latest  : "
    << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]."
    << std::endl;
  }

  mpcMrtInterface_->updatePolicy();
// Evaluate the current policy
  ocs2::vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;

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

  Eigen::Quaternion<ocs2::scalar_t> quat(msg->imu_quat_values[0], msg->imu_quat_values[1], 
      msg->imu_quat_values[2], msg->imu_quat_values[3]);

  for (size_t i = 0; i < msg->contact_flags.size(); ++i) {
    contactFlag_[i] = msg->contact_flags[i];
  }
  //vector3_t eulerAngles = quatToZyx(quat);
  //vector3_t angularVel = {msg->base_angvel_values[0], msg->base_angvel_values[1], msg->base_angvel_values[2]};
  vector3_t angularVel(msg->imu_angvel_values[0], msg->imu_angvel_values[1], msg->imu_angvel_values[2]);
  vector3_t linearAccel(msg->imu_linacc_values[0], msg->imu_linacc_values[1], msg->imu_linacc_values[2]);

  ocs2::vector_t jointPos(msg->joint_position_values.size());
  ocs2::vector_t jointVel(msg->joint_velocity_values.size());
  for (size_t i = 0; i < msg->joint_position_values.size(); ++i) {
      jointPos(i) = msg->joint_position_values[i];
  }
  for (size_t i = 0; i < msg->joint_velocity_values.size(); ++i) {
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

  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time = time;
  ocs2::scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();


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
  msg.actuator_mode = static_cast<uint8_t>(ActuatorMode::NormalPd);

  const ocs2::scalar_t blendAlpha = mpcBlendActive_
                                        ? std::clamp((currentObservation_.time - mpcBlendStartTime_) /
                                                         mpcBlendDuration_,
                                                     ocs2::scalar_t(0.0), ocs2::scalar_t(1.0))
                                        : ocs2::scalar_t(0.0);

  ocs2::vector_t rawTargetJointState = standJointState_;
  ocs2::vector_t rawTargetJointVelocity = ocs2::vector_t::Zero(standJointState_.size());
  ActuatorMode actuatorMode = ActuatorMode::NormalPd;

  switch (controlState_) {
    case ControlState::Hold:
      rawTargetJointState = estopHoldJointState_;
      actuatorMode = ActuatorMode::StrongPd;
      break;
    case ControlState::RecoveryPose:
      rawTargetJointState = recoveryJointState_;
      actuatorMode = ActuatorMode::StrongPd;
      break;
    case ControlState::SitDown: {
      const ocs2::scalar_t sitAlpha = std::clamp((currentObservation_.time - sitDownStartTime_) / sitDownDuration_,
                                                 ocs2::scalar_t(0.0), ocs2::scalar_t(1.0));
      rawTargetJointState = (1.0 - sitAlpha) * sitDownStartJointState_ + sitAlpha * sitJointState_;
      rawTargetJointVelocity = (sitJointState_ - sitDownStartJointState_) / sitDownDuration_;
      actuatorMode = ActuatorMode::StrongPd;
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
      actuatorMode = ActuatorMode::StrongPd;
      break;
    case ControlState::ZeroTorque:
      rawTargetJointState.setZero();
      rawTargetJointVelocity.setZero();
      actuatorMode = ActuatorMode::ZeroTorque;
      break;
    case ControlState::Mpc:
      actuatorMode = ActuatorMode::NormalPd;
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

  msg.actuator_mode = static_cast<uint8_t>(mpcBlendActive_ ? ActuatorMode::StrongPd : actuatorMode);

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
    if (currentObservation_.state.size() == leggedInterface_->getCentroidalModelInfo().stateDim) {
      estopHoldJointState_ = ocs2::centroidal_model::getJointAngles(
          currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
    } else {
      estopHoldJointState_ = standJointState_;
    }
    controlState_ = ControlState::Hold;
    RCLCPP_WARN(node_->get_logger(), "Controller state active: HOLD current joint positions.");
  } else if (command == static_cast<int>(ControlCommand::RecoveryPose)) {
    mpcReleasePending_ = false;
    mpcBlendActive_ = false;
    controlState_ = ControlState::RecoveryPose;
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
    RCLCPP_INFO(node_->get_logger(), "Controller state active: SIT_DOWN.");
  } else if (command == static_cast<int>(ControlCommand::ZeroTorque)) {
    mpcReleasePending_ = false;
    mpcBlendActive_ = false;
    controlState_ = ControlState::ZeroTorque;
    RCLCPP_WARN(node_->get_logger(), "Controller state active: ZERO_TORQUE.");
  } else if (command == static_cast<int>(ControlCommand::ActivateMpc)) {
    mpcReleasePending_ = true;
    mpcBlendActive_ = false;
    mpcReleaseTime_ = currentObservation_.time + mpcReleaseDelay_;
    controlState_ = ControlState::Hold;
    if (currentObservation_.state.size() == leggedInterface_->getCentroidalModelInfo().stateDim) {
      estopHoldJointState_ = ocs2::centroidal_model::getJointAngles(
          currentObservation_.state, leggedInterface_->getCentroidalModelInfo());
    }
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
