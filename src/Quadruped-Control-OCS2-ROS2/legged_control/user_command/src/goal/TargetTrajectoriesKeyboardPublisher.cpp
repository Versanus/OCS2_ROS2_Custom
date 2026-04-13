/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

//#include "ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h"
#include "user_command/goal/TargetTrajectoriesKeyboardPublisher.h"

#include <algorithm>
#include <cmath>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace {

geometry_msgs::msg::Quaternion quaternionFromYawPitchRoll(
    ocs2::scalar_t yaw, ocs2::scalar_t pitch, ocs2::scalar_t roll) {
  const ocs2::scalar_t halfYaw = 0.5 * yaw;
  const ocs2::scalar_t halfPitch = 0.5 * pitch;
  const ocs2::scalar_t halfRoll = 0.5 * roll;

  const ocs2::scalar_t cy = std::cos(halfYaw);
  const ocs2::scalar_t sy = std::sin(halfYaw);
  const ocs2::scalar_t cp = std::cos(halfPitch);
  const ocs2::scalar_t sp = std::sin(halfPitch);
  const ocs2::scalar_t cr = std::cos(halfRoll);
  const ocs2::scalar_t sr = std::sin(halfRoll);

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = cr * cp * cy + sr * sp * sy;
  quaternion.x = sr * cp * cy - cr * sp * sy;
  quaternion.y = cr * sp * cy + sr * cp * sy;
  quaternion.z = cr * cp * sy - sr * sp * cy;
  return quaternion;
}

ocs2::vector_t integrateBodyVelocityPose(const ocs2::vector_t& initialPose,
                                         const ocs2::vector_t& velocityCommand,
                                         ocs2::scalar_t duration,
                                         ocs2::scalar_t desiredHeight) {
  ocs2::vector_t integratedPose = initialPose;
  const ocs2::scalar_t yaw0 = initialPose(3);
  const ocs2::scalar_t vx = velocityCommand(0);
  const ocs2::scalar_t vy = velocityCommand(1);
  const ocs2::scalar_t yawRate = velocityCommand(2);

  if (std::abs(yawRate) < 1e-6) {
    const ocs2::scalar_t bodyDx = vx * duration;
    const ocs2::scalar_t bodyDy = vy * duration;
    integratedPose(0) += std::cos(yaw0) * bodyDx - std::sin(yaw0) * bodyDy;
    integratedPose(1) += std::sin(yaw0) * bodyDx + std::cos(yaw0) * bodyDy;
  } else {
    const ocs2::scalar_t yaw1 = yaw0 + yawRate * duration;
    integratedPose(0) +=
        (vx / yawRate) * (std::sin(yaw1) - std::sin(yaw0)) +
        (vy / yawRate) * (std::cos(yaw1) - std::cos(yaw0));
    integratedPose(1) +=
        (-vx / yawRate) * (std::cos(yaw1) - std::cos(yaw0)) +
        (vy / yawRate) * (std::sin(yaw1) - std::sin(yaw0));
  }

  integratedPose(2) = desiredHeight;
  integratedPose(3) = yaw0 + yawRate * duration;
  integratedPose(4) = 0.0;
  integratedPose(5) = 0.0;
  return integratedPose;
}

}  // namespace



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesKeyboardPublisher::TargetTrajectoriesKeyboardPublisher(
    const rclcpp::Node::SharedPtr& node, const std::string& topicPrefix,
    const ocs2::scalar_array_t& targetCommandLimits,
    //CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun,
    const std::string& referenceFile)
    : node_(node),
      targetCommandLimits_(Eigen::Map<const ocs2::vector_t>(
          targetCommandLimits.data(), targetCommandLimits.size())),
      // commandLineToTargetTrajectoriesFun_(
      //     std::move(commandLineToTargetTrajectoriesFun)),
          defaultJointState_(12) {
  ocs2::loadData::loadCppDataType(referenceFile, "comHeight", comHeight_);
  ocs2::loadData::loadEigenMatrix(referenceFile, "defaultJointState",
                            defaultJointState_);
  ocs2::loadData::loadCppDataType(referenceFile, "targetRotationVelocity",
                            targetRotationVelocity_);
  boost::property_tree::ptree referenceInfoTree;
  boost::property_tree::read_info(referenceFile, referenceInfoTree);
  const double fallbackDisplacementVelocity =
      referenceInfoTree.get<double>("targetDisplacementVelocity", 0.5);
  targetDisplacementVelocityForward_ =
      static_cast<ocs2::scalar_t>(referenceInfoTree.get<double>(
          "targetDisplacementVelocityForward", fallbackDisplacementVelocity));
  targetDisplacementVelocityLateral_ =
      static_cast<ocs2::scalar_t>(referenceInfoTree.get<double>(
          "targetDisplacementVelocityLateral", fallbackDisplacementVelocity));
  velocityPreviewDistanceLowSpeed_ = static_cast<ocs2::scalar_t>(
      referenceInfoTree.get<double>("teleop.velocity_preview_distance_low_speed", 1.0));
  velocityPreviewDistanceHighSpeed_ = static_cast<ocs2::scalar_t>(
      referenceInfoTree.get<double>("teleop.velocity_preview_distance_high_speed", 2.0));
  velocityPreviewSpeedLow_ = static_cast<ocs2::scalar_t>(
      referenceInfoTree.get<double>("teleop.velocity_preview_speed_low", 0.5));
  velocityPreviewSpeedHigh_ = static_cast<ocs2::scalar_t>(
      referenceInfoTree.get<double>("teleop.velocity_preview_speed_high", 4.5));
  
  // observation subscriber
  auto observationCallback =
      [this](const legged_msgs::msg::MpcObservation::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_); 
        //By locking latestObservationMutex_, it ensures that the following code block is thread-safe, 
        //preventing multiple threads from simultaneously accessing or modifying the shared resource latestObservation_.
        latestObservation_ = readObservationMsg(*msg);
      };
  observationSubscriber_ =
      node->create_subscription<legged_msgs::msg::MpcObservation>(
          topicPrefix + "_mpc_observation", 1, observationCallback);

  // // Trajectories publisher
  // targetTrajectoriesPublisherPtr_.reset(
  //     new TargetTrajectoriesRosPublisher(node, topicPrefix));
  targetTrajectoriesPublisherPtr_=
      node_->create_publisher<legged_msgs::msg::MpcTargetTrajectories>(topicPrefix + "_mpc_target", 1); 
  targetTrajectoryPathPublisherPtr_ =
      node_->create_publisher<nav_msgs::msg::Path>(topicPrefix + "_user_command_target_path", 1);

  // Initialize latestObservation_
  latestObservation_.state = Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>::Zero(24);
  latestObservation_.input = Eigen::Matrix<ocs2::scalar_t, Eigen::Dynamic, 1>::Zero(24);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishGoalCommand(const ocs2::vector_t& goalCommand) {
  const ocs2::vector_t commandLineInput = goalCommand.cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);
  std::cout << "Publishing goal command: [" << ocs2::toDelimitedString(commandLineInput) << "]\n";
  resetVelocityReference();
  publishTargetTrajectories(commandLineToTargetTrajectories(commandLineInput, getLatestObservation()));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishVelocityCommand(const ocs2::vector_t& velocityCommand, bool verbose) {
  if (verbose) {
    std::cout << "Publishing velocity command: [" << ocs2::toDelimitedString(velocityCommand) << "]\n";
  }
  publishTargetTrajectories(velocityCommandToTargetTrajectories(velocityCommand, getLatestObservation()), verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishHoldPositionCommand(bool verbose) {
  if (verbose) {
    std::cout << "Publishing hold-position command.\n";
  }
  resetVelocityReference();
  publishTargetTrajectories(holdCurrentPoseToTargetTrajectories(getLatestObservation()), verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t TargetTrajectoriesKeyboardPublisher::adjustDesiredHeight(ocs2::scalar_t deltaHeight) {
  comHeight_ = std::clamp(comHeight_ + deltaHeight, ocs2::scalar_t(0.12), ocs2::scalar_t(0.40));
  return comHeight_;
}

// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// void TargetTrajectoriesKeyboardPublisher::publishKeyboardCommand(
//     const std::string& commadMsg) {
//   while (rclcpp::ok()) {
//     // get command line
//     std::cout << commadMsg << ": ";
//     const ocs2::vector_t commandLineInput = getCommandLine()
//                                           .cwiseMin(targetCommandLimits_)
//                                           .cwiseMax(-targetCommandLimits_); //.cwiseMin and .cwiseMax are from Eigen library

//     // display
//     std::cout << "The following command is published: ["
//               << ocs2::toDelimitedString(commandLineInput) << "]\n\n";

//     std::cout << "success" << ": ";

//     // get the latest observation
//     rclcpp::spin_some(node_->get_node_base_interface());
//     ocs2::SystemObservation observation;
//     {
//       std::lock_guard<std::mutex> lock(latestObservationMutex_);
//       observation = latestObservation_;
//     }
    
//     // get TargetTrajectories
//     const auto targetTrajectories =
//         commandLineToTargetTrajectories(commandLineInput, observation);

//     // publish TargetTrajectories
//     const auto mpcTargetTrajectoriesMsg = createTargetTrajectoriesMsg(targetTrajectories);
//     targetTrajectoriesPublisherPtr_->publish(mpcTargetTrajectoriesMsg);
//   }  // end of while loop
// }


// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// ocs2::vector_t TargetTrajectoriesKeyboardPublisher::getCommandLine() {
//   // get command line as one long string
//   auto shouldTerminate = []() { return !rclcpp::ok(); };
//   const std::string line = ocs2::getCommandLineString(shouldTerminate);
//   // could add if else here to decide gait and goal

//   // a line to words
//   const std::vector<std::string> words = ocs2::stringToWords(line);

//   const size_t targetCommandSize = targetCommandLimits_.size();
//   ocs2::vector_t targetCommand = ocs2::vector_t::Zero(targetCommandSize);
//   for (size_t i = 0; i < std::min(words.size(), targetCommandSize); i++) {
//     targetCommand(i) = static_cast<ocs2::scalar_t>(stof(words[i]));
//   }

//   return targetCommand;
// }


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// transfer legged_msgs::msg::MpcObservation to ocs2::SystemObservation
// the former contains float array, the later contains Eigen::Matrix. The values are same.
ocs2::SystemObservation TargetTrajectoriesKeyboardPublisher::readObservationMsg(
    const legged_msgs::msg::MpcObservation& observationMsg) {
  ocs2::SystemObservation observation;

  observation.time = observationMsg.time;

  const auto& state = observationMsg.state.value;
  observation.state =
      Eigen::Map<const Eigen::VectorXf>(state.data(), state.size())
          .cast<ocs2::scalar_t>();

  const auto& input = observationMsg.input.value;
  observation.input =
      Eigen::Map<const Eigen::VectorXf>(input.data(), input.size())
          .cast<ocs2::scalar_t>();

  observation.mode = observationMsg.mode;

  return observation;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
legged_msgs::msg::MpcTargetTrajectories TargetTrajectoriesKeyboardPublisher::createTargetTrajectoriesMsg(
    const ocs2::TargetTrajectories& targetTrajectories) {
  legged_msgs::msg::MpcTargetTrajectories targetTrajectoriesMsg;
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;

  // time and state
  size_t N = stateTrajectory.size();
  targetTrajectoriesMsg.time_trajectory.resize(N);
  targetTrajectoriesMsg.state_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.time_trajectory[i] = timeTrajectory[i];

    targetTrajectoriesMsg.state_trajectory[i].value = std::vector<float>(
        stateTrajectory[i].data(),
        stateTrajectory[i].data() + stateTrajectory[i].size());
  }  // end of i loop

  // input
  N = inputTrajectory.size();
  targetTrajectoriesMsg.input_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.input_trajectory[i].value = std::vector<float>(
        inputTrajectory[i].data(),
        inputTrajectory[i].data() + inputTrajectory[i].size());
  }  // end of i loop

  return targetTrajectoriesMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::SystemObservation TargetTrajectoriesKeyboardPublisher::getLatestObservation() {
  rclcpp::spin_some(node_->get_node_base_interface());
  std::lock_guard<std::mutex> lock(latestObservationMutex_);
  return latestObservation_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
nav_msgs::msg::Path TargetTrajectoriesKeyboardPublisher::createTargetPathMsg(
    const ocs2::TargetTrajectories& targetTrajectories) const {
  nav_msgs::msg::Path pathMsg;
  pathMsg.header.stamp = node_->now();
  pathMsg.header.frame_id = targetPathFrameId_;
  pathMsg.poses.reserve(targetTrajectories.stateTrajectory.size());

  for (const auto& state : targetTrajectories.stateTrajectory) {
    if (state.size() < 12) {
      continue;
    }

    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header = pathMsg.header;
    poseStamped.pose.position.x = state(6);
    poseStamped.pose.position.y = state(7);
    poseStamped.pose.position.z = state(8);
    poseStamped.pose.orientation = quaternionFromYawPitchRoll(state(9), state(10), state(11));
    pathMsg.poses.push_back(poseStamped);
  }

  return pathMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishTargetTrajectories(const ocs2::TargetTrajectories& targetTrajectories, bool verbose) {
  const auto mpcTargetTrajectoriesMsg = createTargetTrajectoriesMsg(targetTrajectories);
  targetTrajectoriesPublisherPtr_->publish(mpcTargetTrajectoriesMsg);
  targetTrajectoryPathPublisherPtr_->publish(createTargetPathMsg(targetTrajectories));
  if (verbose) {
    std::cout << "Target trajectory publish succeed!!!.\n";
    std::cout << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::resetVelocityReference() {
  velocityReferenceInitialized_ = false;
  lastVelocityReferenceTime_ = 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t TargetTrajectoriesKeyboardPublisher::computeVelocityPreviewDistance(
    const ocs2::vector_t& velocityCommand) const {
  const ocs2::scalar_t planarSpeed = velocityCommand.head<2>().norm();
  const ocs2::scalar_t clampedSpeed =
      std::clamp(planarSpeed, velocityPreviewSpeedLow_, velocityPreviewSpeedHigh_);

  ocs2::scalar_t interpolation = 0.0;
  if (velocityPreviewSpeedHigh_ > velocityPreviewSpeedLow_) {
    interpolation = (clampedSpeed - velocityPreviewSpeedLow_) /
                    (velocityPreviewSpeedHigh_ - velocityPreviewSpeedLow_);
  }

  return
      velocityPreviewDistanceLowSpeed_ +
      interpolation * (velocityPreviewDistanceHighSpeed_ - velocityPreviewDistanceLowSpeed_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::scalar_t TargetTrajectoriesKeyboardPublisher::estimateTimeToTarget(const ocs2::vector_t& desiredBaseDisplacement) {
  const ocs2::scalar_t& dx = desiredBaseDisplacement(0);
  const ocs2::scalar_t& dy = desiredBaseDisplacement(1);
  const ocs2::scalar_t& dz = desiredBaseDisplacement(2);
  const ocs2::scalar_t& dyaw = desiredBaseDisplacement(3);
  const ocs2::scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity_;
  const ocs2::scalar_t forwardTime = std::abs(dx) / targetDisplacementVelocityForward_;
  const ocs2::scalar_t lateralTime = std::abs(dy) / targetDisplacementVelocityLateral_;
  const ocs2::scalar_t verticalTime = std::abs(dz) / targetDisplacementVelocityForward_;
  return std::max({rotationTime, forwardTime, lateralTime, verticalTime, minGoalTrajectoryDuration_});
}

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw]
 * @param [in] observation : the current observation
 */
ocs2::TargetTrajectories TargetTrajectoriesKeyboardPublisher::commandLineToTargetTrajectories(
    const ocs2::vector_t& commadLineTarget, const ocs2::SystemObservation& observation) {
      
  const ocs2::vector_t currentPose = observation.state.segment<6>(6);
  const ocs2::vector_t targetPose = [&]() {
    ocs2::vector_t target(6);
    // base p_x, p_y are relative to current state
    target(0) = currentPose(0) + commadLineTarget(0);
    target(1) = currentPose(1) + commadLineTarget(1);
    // base z relative to the default height
    target(2) = comHeight_ + commadLineTarget(2);
    // theta_z relative to current
    target(3) = currentPose(3) + commadLineTarget(3) * M_PI / 180.0;
    // theta_y, theta_x
    target(4) = currentPose(4);
    target(5) = currentPose(5);
    return target;
  }();
  
  // target reaching duration
  const ocs2::scalar_t targetReachingTime =
      observation.time + estimateTimeToTarget(targetPose - currentPose);

  // desired time trajectory
  const ocs2::scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  ocs2::vector_array_t stateTrajectory(2, ocs2::vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << ocs2::vector_t::Zero(6), currentPose, defaultJointState_;
  stateTrajectory[1] << ocs2::vector_t::Zero(6), targetPose, defaultJointState_;

  // desired input trajectory (just right dimensions, they are not used)
  const ocs2::vector_array_t inputTrajectory(
      2, ocs2::vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::TargetTrajectories TargetTrajectoriesKeyboardPublisher::velocityCommandToTargetTrajectories(
    const ocs2::vector_t& velocityCommand, const ocs2::SystemObservation& observation) {
  const ocs2::vector_t currentPose = observation.state.segment<6>(6);
  ocs2::vector_t desiredPoseNow = currentPose;
  desiredPoseNow(2) = comHeight_;
  desiredPoseNow(4) = 0.0;
  desiredPoseNow(5) = 0.0;
  if (!velocityReferenceInitialized_ || observation.time < lastVelocityReferenceTime_ ||
      observation.time - lastVelocityReferenceTime_ > 0.5) {
    velocityReferencePose_ = desiredPoseNow;
    lastVelocityReferenceTime_ = observation.time;
    velocityReferenceInitialized_ = true;
  }

  const ocs2::scalar_t dt = std::clamp(observation.time - lastVelocityReferenceTime_, ocs2::scalar_t(0.0), ocs2::scalar_t(0.1));
  velocityReferencePose_ = integrateBodyVelocityPose(velocityReferencePose_, velocityCommand, dt, comHeight_);
  lastVelocityReferenceTime_ = observation.time;

  const ocs2::scalar_t velocityPreviewDistance =
      computeVelocityPreviewDistance(velocityCommand);
  const ocs2::scalar_t planarSpeed = velocityCommand.head<2>().norm();
  const ocs2::scalar_t yawSpeed = std::abs(velocityCommand(2));
  const ocs2::scalar_t rawEquivalentProgressSpeed =
      std::max(planarSpeed, ocs2::scalar_t(0.5) * yawSpeed);
  const ocs2::scalar_t equivalentProgressSpeed =
      std::clamp(rawEquivalentProgressSpeed, velocityPreviewSpeedLow_, velocityPreviewSpeedHigh_);
  const ocs2::scalar_t velocityPreviewTime =
      std::max(velocityPreviewDistance / equivalentProgressSpeed, minGoalTrajectoryDuration_);
  const int numTrajectorySamples =
      std::clamp(static_cast<int>(std::ceil(velocityPreviewTime / 0.1)) + 1, 3, 25);
  const ocs2::vector_t referenceStartPose = velocityReferencePose_;

  ocs2::scalar_array_t timeTrajectory;
  timeTrajectory.reserve(numTrajectorySamples);
  ocs2::vector_array_t stateTrajectory;
  stateTrajectory.reserve(numTrajectorySamples);
  ocs2::vector_array_t inputTrajectory;
  inputTrajectory.reserve(numTrajectorySamples);

  for (int i = 0; i < numTrajectorySamples; ++i) {
    const ocs2::scalar_t interpolation =
        numTrajectorySamples == 1 ? 0.0 : static_cast<ocs2::scalar_t>(i) / static_cast<ocs2::scalar_t>(numTrajectorySamples - 1);
    const ocs2::scalar_t sampleTimeOffset = interpolation * velocityPreviewTime;
    const ocs2::scalar_t sampleTime = observation.time + sampleTimeOffset;
    const ocs2::vector_t samplePose =
        integrateBodyVelocityPose(referenceStartPose, velocityCommand, sampleTimeOffset, comHeight_);

    timeTrajectory.push_back(sampleTime);
    ocs2::vector_t state = ocs2::vector_t::Zero(observation.state.size());
    state << ocs2::vector_t::Zero(6), samplePose, defaultJointState_;
    stateTrajectory.push_back(state);
    inputTrajectory.emplace_back(ocs2::vector_t::Zero(observation.input.size()));
  }

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::TargetTrajectories TargetTrajectoriesKeyboardPublisher::holdCurrentPoseToTargetTrajectories(
    const ocs2::SystemObservation& observation) {
  const ocs2::vector_t currentPose = observation.state.segment<6>(6);
  const ocs2::scalar_array_t timeTrajectory{observation.time, observation.time + holdTrajectoryDuration_};
  ocs2::vector_t desiredPoseNow = currentPose;
  desiredPoseNow(2) = comHeight_;
  desiredPoseNow(4) = 0.0;
  desiredPoseNow(5) = 0.0;
  ocs2::vector_t holdPose = currentPose;
  holdPose(2) = comHeight_;
  holdPose(4) = 0.0;
  holdPose(5) = 0.0;

  ocs2::vector_array_t stateTrajectory(2, ocs2::vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << ocs2::vector_t::Zero(6), desiredPoseNow, defaultJointState_;
  stateTrajectory[1] << ocs2::vector_t::Zero(6), holdPose, defaultJointState_;

  const ocs2::vector_array_t inputTrajectory(2, ocs2::vector_t::Zero(observation.input.size()));
  return {timeTrajectory, stateTrajectory, inputTrajectory};
}
