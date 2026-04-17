#include "motion_control/controller/RlBackend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

#include "motion_control/controller/OrtCpuRunner.h"

namespace {

constexpr double kStrongKpRatio = 4.0;
constexpr double kStrongKdRatio = 2.0;
constexpr std::size_t kJointCount = 12;

std::filesystem::path resolvePath(const std::filesystem::path& baseFile, const std::string& candidate) {
  const std::filesystem::path candidatePath(candidate);
  if (candidatePath.is_absolute()) {
    return candidatePath;
  }

  return std::filesystem::weakly_canonical(baseFile.parent_path() / candidatePath);
}

template <typename T>
T clampMagnitude(T value, T limit) {
  return std::clamp(value, -limit, limit);
}

double smoothStep01(double alpha) {
  const double clampedAlpha = std::clamp(alpha, 0.0, 1.0);
  return clampedAlpha * clampedAlpha * (3.0 - 2.0 * clampedAlpha);
}

bool isFiniteVector(const std::vector<double>& values, std::size_t expectedSize) {
  if (values.size() < expectedSize) {
    return false;
  }

  return std::all_of(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(expectedSize), [](double value) {
    return std::isfinite(value);
  });
}

Eigen::Quaterniond normalizedQuaternionFromState(const legged_msgs::msg::SimulatorStateData& state) {
  if (state.base_quat_values.size() < 4) {
    throw std::runtime_error("State message does not contain a valid base quaternion.");
  }

  Eigen::Quaterniond quaternion(
      state.base_quat_values[0],
      state.base_quat_values[1],
      state.base_quat_values[2],
      state.base_quat_values[3]);
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < 1e-6) {
    throw std::runtime_error("State message contains a degenerate base quaternion.");
  }

  quaternion.normalize();
  return quaternion;
}

Eigen::Quaterniond normalizedQuaternionFromSensor(const legged_msgs::msg::SimulatorSensorData& sensor) {
  if (sensor.imu_quat_values.size() < 4) {
    throw std::runtime_error("Sensor message does not contain a valid IMU quaternion.");
  }

  Eigen::Quaterniond quaternion(
      sensor.imu_quat_values[0],
      sensor.imu_quat_values[1],
      sensor.imu_quat_values[2],
      sensor.imu_quat_values[3]);
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || norm < 1e-6) {
    throw std::runtime_error("Sensor message contains a degenerate IMU quaternion.");
  }

  quaternion.normalize();
  return quaternion;
}

}  // namespace

RlBackend::RlBackend(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      controllerConfig_(),
      settings_(),
      policyRunner_(),
      jointControlPublisher_(),
      emergencyOverrideStatePublisher_(),
      simulatorStateSubscriber_(),
      simulatorSensorSubscriber_(),
      velocityCommandSubscriber_(),
      emergencyOverrideSubscriber_(),
      policyTimer_(),
      latestState_(),
      latestSensor_(),
      latestVelocityCommand_(),
      lastStateReceiptTime_(0, 0, RCL_ROS_TIME),
      lastSensorReceiptTime_(0, 0, RCL_ROS_TIME),
      lastCommandReceiptTime_(0, 0, RCL_ROS_TIME),
      firstFreshStateTime_(0, 0, RCL_ROS_TIME),
      hasState_(false),
      hasSensor_(false),
      startupHoldStarted_(false),
      hasHoldPose_(false),
      controlState_(ControlState::Hold),
      poseTransitionStartTime_(0, 0, RCL_ROS_TIME),
      poseTransitionDurationSec_(0.0),
      defaultJointState_(ocs2::vector_t::Zero(kJointCount)),
      holdJointState_(ocs2::vector_t::Zero(kJointCount)),
      poseTransitionStartPose_(ocs2::vector_t::Zero(kJointCount)),
      poseTransitionTargetPose_(ocs2::vector_t::Zero(kJointCount)),
      standJointState_(ocs2::vector_t::Zero(kJointCount)),
      recoveryJointState_(ocs2::vector_t::Zero(kJointCount)),
      sitJointState_(ocs2::vector_t::Zero(kJointCount)),
      commandObservationScale_(ocs2::vector_t::Ones(3)),
      lastAction_(kJointCount, 0.0f) {}

RlBackend::~RlBackend() = default;

bool RlBackend::configure(const ControllerConfig& config) {
  if (!node_) {
    return false;
  }
  if (config.referenceFile.empty() || config.rlConfigFile.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RL backend requires non-empty referenceFile and rlConfigFile parameters.");
    return false;
  }

  controllerConfig_ = config;
  if (!loadSettings(config)) {
    return false;
  }
  if (!loadReferencePoses(config.referenceFile)) {
    return false;
  }

  policyRunner_ = std::make_unique<OrtCpuRunner>();
  if (!policyRunner_->load(settings_.modelPath, settings_.inputName, settings_.outputName)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load RL model '%s'.", settings_.modelPath.c_str());
    return false;
  }

  if (policyRunner_->inputSize() != settings_.observationDim || policyRunner_->outputSize() != settings_.actionDim) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RL model I/O mismatch. input=%zu expected=%zu output=%zu expected=%zu.",
                 policyRunner_->inputSize(), settings_.observationDim, policyRunner_->outputSize(), settings_.actionDim);
    return false;
  }

  lastAction_.assign(settings_.actionDim, 0.0f);

  RCLCPP_INFO(node_->get_logger(),
              "Configured RL backend with model='%s' input='%s'(%zu) output='%s'(%zu) policy_hz=%.2f action_scale=%.3f.",
              settings_.modelPath.c_str(), policyRunner_->inputName().c_str(), policyRunner_->inputSize(),
              policyRunner_->outputName().c_str(), policyRunner_->outputSize(),
              settings_.policyHz, settings_.actionScale);
  return true;
}

void RlBackend::launch() {
  setupRosInterfaces();
  RCLCPP_INFO(node_->get_logger(),
              "RL backend starts in hold. Enter mode:vel, press '1' for stance PD, then press '2' to arm RL policy output.");

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, settings_.policyHz));
  policyTimer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RlBackend::policyTimerCallback, this));

  publishEmergencyOverrideState();
  rclcpp::spin(node_);
}

const char* RlBackend::name() const {
  return "rl";
}

bool RlBackend::loadSettings(const ControllerConfig& config) {
  namespace fs = std::filesystem;

  try {
    const fs::path rlConfigPath = fs::weakly_canonical(config.rlConfigFile);
    if (!fs::exists(rlConfigPath)) {
      throw std::runtime_error("RL config file does not exist.");
    }

    boost::property_tree::ptree tree;
    boost::property_tree::read_info(rlConfigPath.string(), tree);

    settings_.modelPath = resolvePath(rlConfigPath, tree.get<std::string>("onnxModelPath")).string();
    settings_.inputName = tree.get<std::string>("onnxInputName", settings_.inputName);
    settings_.outputName = tree.get<std::string>("onnxOutputName", settings_.outputName);
    settings_.policyHz = tree.get<double>("policyHz", settings_.policyHz);
    settings_.actionScale = tree.get<double>("actionScale", settings_.actionScale);
    settings_.actionClip = tree.get<double>("actionClip", settings_.actionClip);
    settings_.kpRatio = tree.get<double>("kpRatio", settings_.kpRatio);
    settings_.kdRatio = tree.get<double>("kdRatio", settings_.kdRatio);
    settings_.stateTimeoutSec = tree.get<double>("stateTimeoutSec", settings_.stateTimeoutSec);
    settings_.commandTimeoutSec = tree.get<double>("commandTimeoutSec", settings_.commandTimeoutSec);
    settings_.startupHoldSec = tree.get<double>("startupHoldSec", settings_.startupHoldSec);
    settings_.standTransitionSec = tree.get<double>("standTransitionSec", settings_.standTransitionSec);
    settings_.sitTransitionSec = tree.get<double>("sitTransitionSec", settings_.sitTransitionSec);
    settings_.commandActivationThreshold =
        tree.get<double>("commandActivationThreshold", settings_.commandActivationThreshold);
    settings_.policyCommandMaxX = tree.get<double>("policyCommandMaxX", settings_.policyCommandMaxX);
    settings_.policyCommandMaxY = tree.get<double>("policyCommandMaxY", settings_.policyCommandMaxY);
    settings_.policyCommandMaxYaw = tree.get<double>("policyCommandMaxYaw", settings_.policyCommandMaxYaw);
    settings_.requireCommandForPolicy =
        tree.get<bool>("requireCommandForPolicy", settings_.requireCommandForPolicy);
    settings_.observationDim = static_cast<std::size_t>(tree.get<int>("obsDim", static_cast<int>(settings_.observationDim)));
    settings_.actionDim = static_cast<std::size_t>(tree.get<int>("actDim", static_cast<int>(settings_.actionDim)));

    if (settings_.modelPath.empty()) {
      throw std::runtime_error("onnxModelPath must not be empty.");
    }
    if (!fs::exists(settings_.modelPath)) {
      throw std::runtime_error("Resolved ONNX model path does not exist: " + settings_.modelPath);
    }
    if (settings_.observationDim == 0 || settings_.actionDim == 0) {
      throw std::runtime_error("obsDim and actDim must be positive.");
    }
    if (!std::isfinite(settings_.standTransitionSec) || settings_.standTransitionSec < 0.0) {
      throw std::runtime_error("standTransitionSec must be finite and non-negative.");
    }
    if (!std::isfinite(settings_.sitTransitionSec) || settings_.sitTransitionSec < 0.0) {
      throw std::runtime_error("sitTransitionSec must be finite and non-negative.");
    }
    if (!std::isfinite(settings_.policyCommandMaxX) || !std::isfinite(settings_.policyCommandMaxY) ||
        !std::isfinite(settings_.policyCommandMaxYaw)) {
      throw std::runtime_error("policyCommandMaxX/Y/Yaw must be finite.");
    }
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load RL config '%s': %s", config.rlConfigFile.c_str(), error.what());
    return false;
  }
}

bool RlBackend::loadReferencePoses(const std::string& referenceFile) {
  try {
    ocs2::loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState_);
    standJointState_ = defaultJointState_;
    recoveryJointState_ = defaultJointState_;
    sitJointState_.setZero(defaultJointState_.size());
    ocs2::loadData::loadEigenMatrix(referenceFile, "standJointState", standJointState_);
    ocs2::loadData::loadEigenMatrix(referenceFile, "recoveryJointState", recoveryJointState_);
    ocs2::loadData::loadEigenMatrix(referenceFile, "sitJointState", sitJointState_);

    boost::property_tree::ptree referenceTree;
    boost::property_tree::read_info(referenceFile, referenceTree);
    const double fallbackForwardVelocity = referenceTree.get<double>("targetDisplacementVelocity", 0.5);
    const double referenceForwardVelocity =
        referenceTree.get<double>("targetDisplacementVelocityForward", fallbackForwardVelocity);
    const double referenceLateralVelocity =
        referenceTree.get<double>("targetDisplacementVelocityLateral", fallbackForwardVelocity);
    const double referenceYawVelocity = referenceTree.get<double>("targetRotationVelocity", 0.6);
    const double commandAxisX = referenceTree.get<double>("command_axis.x", 1.0);
    const double commandAxisY = referenceTree.get<double>("command_axis.y", 1.0);
    const double commandAxisYaw = referenceTree.get<double>("command_axis.yaw", 1.0);

    const auto computeCommandScale = [](double policyMax, double referenceMax, double axisScale) {
      const double denominator = referenceMax * axisScale;
      if (!std::isfinite(policyMax) || !std::isfinite(referenceMax) || !std::isfinite(axisScale) ||
          std::abs(denominator) < 1e-9) {
        throw std::runtime_error("Invalid reference teleop scaling while computing RL command remap.");
      }
      return policyMax / denominator;
    };

    commandObservationScale_ =
        (ocs2::vector_t(3)
             << computeCommandScale(settings_.policyCommandMaxX, referenceForwardVelocity, commandAxisX),
                computeCommandScale(settings_.policyCommandMaxY, referenceLateralVelocity, commandAxisY),
                computeCommandScale(settings_.policyCommandMaxYaw, referenceYawVelocity, commandAxisYaw))
                .finished();

    RCLCPP_INFO(node_->get_logger(),
                "RL command remap derived from reference.info: raw->policy scale = [%.3f, %.3f, %.3f].",
                commandObservationScale_(0), commandObservationScale_(1), commandObservationScale_(2));
    return defaultJointState_.size() == static_cast<Eigen::Index>(kJointCount);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load RL reference poses from '%s': %s",
                 referenceFile.c_str(), error.what());
    return false;
  }
}

void RlBackend::setupRosInterfaces() {
  jointControlPublisher_ = node_->create_publisher<legged_msgs::msg::JointControlData>("joint_control_data", 1);
  emergencyOverrideStatePublisher_ = node_->create_publisher<std_msgs::msg::Int32>(
      controllerConfig_.robotName + "_emergency_override_state", 1);
  simulatorStateSubscriber_ = node_->create_subscription<legged_msgs::msg::SimulatorStateData>(
      "simulator_state_data", 1, std::bind(&RlBackend::stateCallback, this, std::placeholders::_1));
  simulatorSensorSubscriber_ = node_->create_subscription<legged_msgs::msg::SimulatorSensorData>(
      "simulator_sensor_data", 1, std::bind(&RlBackend::sensorCallback, this, std::placeholders::_1));
  velocityCommandSubscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      controllerConfig_.robotName + "_velocity_command", 1,
      std::bind(&RlBackend::velocityCommandCallback, this, std::placeholders::_1));
  emergencyOverrideSubscriber_ = node_->create_subscription<std_msgs::msg::Int32>(
      controllerConfig_.robotName + "_emergency_override", 1,
      std::bind(&RlBackend::emergencyOverrideCallback, this, std::placeholders::_1));
}

void RlBackend::stateCallback(const legged_msgs::msg::SimulatorStateData::SharedPtr msg) {
  latestState_ = *msg;
  hasState_ = true;
  lastStateReceiptTime_ = node_->now();

  if (controlState_ == ControlState::Hold) {
    latchHoldPoseFromLatestState();
  }
}

void RlBackend::sensorCallback(const legged_msgs::msg::SimulatorSensorData::SharedPtr msg) {
  latestSensor_ = *msg;
  hasSensor_ = true;
  lastSensorReceiptTime_ = node_->now();
}

void RlBackend::velocityCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  latestVelocityCommand_ = *msg;
  lastCommandReceiptTime_ = node_->now();
}

void RlBackend::emergencyOverrideCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  switch (msg->data) {
    case static_cast<int>(ControlCommand::ActivatePolicyLegacy):
    case static_cast<int>(ControlCommand::ActivatePolicy):
      controlState_ = ControlState::Policy;
      startupHoldStarted_ = false;
      resetPoseTransition();
      lastAction_.assign(settings_.actionDim, 0.0f);
      RCLCPP_INFO(node_->get_logger(), "RL backend activated.");
      break;
    case static_cast<int>(ControlCommand::Hold):
      controlState_ = ControlState::Hold;
      resetPoseTransition();
      latchHoldPoseFromLatestState();
      lastAction_.assign(settings_.actionDim, 0.0f);
      RCLCPP_WARN(node_->get_logger(), "RL backend switched to hold pose.");
      break;
    case static_cast<int>(ControlCommand::RecoveryPose):
      controlState_ = ControlState::RecoveryPose;
      resetPoseTransition();
      lastAction_.assign(settings_.actionDim, 0.0f);
      RCLCPP_WARN(node_->get_logger(), "RL backend switched to recovery pose.");
      break;
    case static_cast<int>(ControlCommand::SitDown):
      controlState_ = ControlState::SitDown;
      startupHoldStarted_ = false;
      beginPoseTransition(sitJointState_, settings_.sitTransitionSec);
      lastAction_.assign(settings_.actionDim, 0.0f);
      RCLCPP_WARN(node_->get_logger(),
                  "RL backend switched to sit pose. Sitting down over %.2f s with strong PD ratios kp=%.1f kd=%.1f.",
                  settings_.sitTransitionSec, kStrongKpRatio, kStrongKdRatio);
      break;
    case static_cast<int>(ControlCommand::ZeroTorque):
      controlState_ = ControlState::ZeroTorque;
      resetPoseTransition();
      lastAction_.assign(settings_.actionDim, 0.0f);
      RCLCPP_WARN(node_->get_logger(), "RL backend switched to zero torque.");
      break;
    case static_cast<int>(ControlCommand::Stand):
      controlState_ = ControlState::Stand;
      startupHoldStarted_ = false;
      beginPoseTransition(standJointState_, settings_.standTransitionSec);
      lastAction_.assign(settings_.actionDim, 0.0f);
      RCLCPP_INFO(node_->get_logger(),
                  "RL backend switched to stance PD. Standing up over %.2f s with strong PD ratios kp=%.1f kd=%.1f.",
                  settings_.standTransitionSec, kStrongKpRatio, kStrongKdRatio);
      break;
    default:
      RCLCPP_WARN(node_->get_logger(), "Ignoring unknown RL emergency override command %d.", msg->data);
      break;
  }

  publishEmergencyOverrideState();
}

void RlBackend::policyTimerCallback() {
  if (controlState_ == ControlState::ZeroTorque) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    publishZeroTorqueCommand();
    return;
  }

  if (controlState_ == ControlState::Hold) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (hasFreshState()) {
      latchHoldPoseFromLatestState();
    }
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, kStrongKpRatio, kStrongKdRatio);
    }
    return;
  }

  if (controlState_ == ControlState::Stand) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (poseTransitionStartTime_.nanoseconds() == 0) {
      beginPoseTransition(standJointState_, settings_.standTransitionSec);
    }
    publishPoseCommand(transitionPoseCommand(), kStrongKpRatio, kStrongKdRatio);
    return;
  }

  if (controlState_ == ControlState::SitDown) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (poseTransitionStartTime_.nanoseconds() == 0) {
      beginPoseTransition(sitJointState_, settings_.sitTransitionSec);
    }
    publishPoseCommand(transitionPoseCommand(), kStrongKpRatio, kStrongKdRatio);
    return;
  }

  if (controlState_ != ControlState::Policy) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    publishPoseCommand(poseForControlState(), kStrongKpRatio, kStrongKdRatio);
    return;
  }

  if (!hasFreshState() || !hasFreshSensor()) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "RL backend lost fresh simulator state/sensor data. Holding last latched pose.");
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, kStrongKpRatio, kStrongKdRatio);
    }
    return;
  }

  const auto now = node_->now();
  if (!startupHoldStarted_) {
    startupHoldStarted_ = true;
    firstFreshStateTime_ = now;
    latchHoldPoseFromLatestState();
    lastAction_.assign(settings_.actionDim, 0.0f);
    RCLCPP_INFO(node_->get_logger(),
                "Fresh simulator state and sensor data received. Holding current pose for %.2f s before allowing RL policy output.",
                settings_.startupHoldSec);
  }

  if ((now - firstFreshStateTime_).seconds() < settings_.startupHoldSec) {
    latchHoldPoseFromLatestState();
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, kStrongKpRatio, kStrongKdRatio);
    }
    return;
  }

  if (settings_.requireCommandForPolicy && !hasActiveVelocityCommand()) {
    latchHoldPoseFromLatestState();
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, settings_.kpRatio, settings_.kdRatio);
    }
    return;
  }

  const auto computeStart = std::chrono::steady_clock::now();
  std::vector<float> observation;
  if (!buildObservation(observation)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to build RL observation. Holding the last latched pose.");
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, kStrongKpRatio, kStrongKdRatio);
    } else {
      publishPoseCommand(standJointState_, kStrongKpRatio, kStrongKdRatio);
    }
    return;
  }

  const auto inferStart = std::chrono::steady_clock::now();
  std::vector<float> action;
  if (!policyRunner_->infer(observation, action) || action.size() != settings_.actionDim) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "RL inference failed or returned an unexpected action size. Holding the last latched pose.");
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, kStrongKpRatio, kStrongKdRatio);
    } else {
      publishPoseCommand(standJointState_, kStrongKpRatio, kStrongKdRatio);
    }
    return;
  }
  if (!std::all_of(action.begin(), action.end(), [](float value) { return std::isfinite(value); })) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "RL inference produced non-finite actions. Holding the last latched pose.");
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, kStrongKpRatio, kStrongKdRatio);
    } else {
      publishPoseCommand(standJointState_, kStrongKpRatio, kStrongKdRatio);
    }
    return;
  }

  const auto velocityCommand = policyVelocityCommand();
  double maxAbsAction = 0.0;
  for (const float value : action) {
    maxAbsAction = std::max(maxAbsAction, std::abs(static_cast<double>(value)));
  }
  const double inferMs =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - inferStart).count();
  const double computeMs =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - computeStart).count();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                       "RL policy active: vx=%.3f vy=%.3f wz=%.3f compute=%.3f ms infer=%.3f ms max|action|=%.3f",
                       velocityCommand.linear.x, velocityCommand.linear.y, velocityCommand.angular.z,
                       computeMs, inferMs, maxAbsAction);

  publishPolicyCommand(action);
}

bool RlBackend::hasFreshState() const {
  if (!hasState_) {
    return false;
  }

  const auto age = (node_->now() - lastStateReceiptTime_).seconds();
  return std::isfinite(age) && age <= settings_.stateTimeoutSec && isFiniteVector(latestState_.joint_position_values, kJointCount) &&
         isFiniteVector(latestState_.joint_velocity_values, kJointCount) &&
         isFiniteVector(latestState_.base_pose_values, 3) &&
         isFiniteVector(latestState_.base_quat_values, 4) &&
         isFiniteVector(latestState_.base_angvel_values, 3) &&
         isFiniteVector(latestState_.base_linvel_values, 3);
}

bool RlBackend::hasFreshSensor() const {
  if (!hasSensor_) {
    return false;
  }

  const auto age = (node_->now() - lastSensorReceiptTime_).seconds();
  return std::isfinite(age) && age <= settings_.stateTimeoutSec &&
         isFiniteVector(latestSensor_.imu_quat_values, 4) &&
         isFiniteVector(latestSensor_.imu_angvel_values, 3) &&
         isFiniteVector(latestSensor_.local_linvel_values, 3) &&
         isFiniteVector(latestSensor_.joint_position_values, kJointCount) &&
         isFiniteVector(latestSensor_.joint_velocity_values, kJointCount);
}

bool RlBackend::hasActiveVelocityCommand() const {
  const auto command = policyVelocityCommand();
  const double commandMagnitude =
      std::abs(command.linear.x) + std::abs(command.linear.y) + std::abs(command.angular.z);
  return std::isfinite(commandMagnitude) && commandMagnitude > settings_.commandActivationThreshold;
}

bool RlBackend::latchHoldPoseFromLatestState() {
  if (!hasFreshState()) {
    return false;
  }

  if (!fillPoseFromLatestState(holdJointState_)) {
    return false;
  }

  hasHoldPose_ = true;
  return true;
}

bool RlBackend::fillPoseFromLatestState(ocs2::vector_t& pose) const {
  if (!hasFreshState()) {
    return false;
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    pose[static_cast<Eigen::Index>(i)] = latestState_.joint_position_values[i];
  }
  return true;
}

void RlBackend::resetPoseTransition() {
  poseTransitionStartTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  poseTransitionDurationSec_ = 0.0;
}

void RlBackend::beginPoseTransition(const ocs2::vector_t& targetPose, double durationSec) {
  if (fillPoseFromLatestState(poseTransitionStartPose_)) {
    holdJointState_ = poseTransitionStartPose_;
    hasHoldPose_ = true;
  } else if (hasHoldPose_) {
    poseTransitionStartPose_ = holdJointState_;
  } else {
    poseTransitionStartPose_ = targetPose;
  }

  poseTransitionTargetPose_ = targetPose;
  poseTransitionDurationSec_ = std::max(0.0, durationSec);
  poseTransitionStartTime_ = node_->now();
}

ocs2::vector_t RlBackend::transitionPoseCommand() const {
  if (poseTransitionStartTime_.nanoseconds() == 0 || poseTransitionDurationSec_ <= 0.0) {
    return poseTransitionTargetPose_;
  }

  const double elapsedSec = (node_->now() - poseTransitionStartTime_).seconds();
  const double alpha = smoothStep01(elapsedSec / poseTransitionDurationSec_);
  return (1.0 - alpha) * poseTransitionStartPose_ + alpha * poseTransitionTargetPose_;
}

bool RlBackend::buildObservation(std::vector<float>& observation) const {
  if (!hasFreshSensor()) {
    return false;
  }

  try {
    const Eigen::Quaterniond quaternion = normalizedQuaternionFromSensor(latestSensor_);
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const Eigen::Vector3d projectedGravity = rotation.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
    const geometry_msgs::msg::Twist velocityCommand = policyVelocityCommand();

    observation.clear();
    observation.reserve(settings_.observationDim);

    for (int i = 0; i < 3; ++i) {
      observation.push_back(static_cast<float>(latestSensor_.local_linvel_values[i]));
    }
    for (int i = 0; i < 3; ++i) {
      observation.push_back(static_cast<float>(latestSensor_.imu_angvel_values[i]));
    }
    for (int i = 0; i < 3; ++i) {
      observation.push_back(static_cast<float>(projectedGravity[i]));
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      observation.push_back(static_cast<float>(latestSensor_.joint_position_values[i] - defaultJointState_[i]));
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      observation.push_back(static_cast<float>(latestSensor_.joint_velocity_values[i]));
    }
    observation.insert(observation.end(), lastAction_.begin(), lastAction_.end());
    observation.push_back(static_cast<float>(velocityCommand.linear.x));
    observation.push_back(static_cast<float>(velocityCommand.linear.y));
    observation.push_back(static_cast<float>(velocityCommand.angular.z));

    if (observation.size() != settings_.observationDim) {
      return false;
    }

    return std::all_of(observation.begin(), observation.end(), [](float value) { return std::isfinite(value); });
  } catch (...) {
    return false;
  }
}

void RlBackend::publishPolicyCommand(const std::vector<float>& action) {
  legged_msgs::msg::JointControlData msg;
  msg.joint_position.resize(kJointCount);
  msg.joint_velocity.assign(kJointCount, 0.0);
  msg.joint_torque.assign(kJointCount, 0.0);
  msg.kp = settings_.kpRatio;
  msg.kd = settings_.kdRatio;

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double clippedAction = clampMagnitude(static_cast<double>(action[i]), settings_.actionClip);
    const double jointTarget = defaultJointState_[i] + settings_.actionScale * clippedAction;
    msg.joint_position[i] = jointTarget;
    lastAction_[i] = static_cast<float>(clippedAction);
  }

  jointControlPublisher_->publish(msg);
  publishEmergencyOverrideState();
}

void RlBackend::publishPoseCommand(const ocs2::vector_t& pose, double kpRatio, double kdRatio) {
  legged_msgs::msg::JointControlData msg;
  msg.joint_position.resize(kJointCount);
  msg.joint_velocity.assign(kJointCount, 0.0);
  msg.joint_torque.assign(kJointCount, 0.0);
  msg.kp = kpRatio;
  msg.kd = kdRatio;

  for (std::size_t i = 0; i < kJointCount; ++i) {
    msg.joint_position[i] = pose[static_cast<Eigen::Index>(i)];
  }

  jointControlPublisher_->publish(msg);
  publishEmergencyOverrideState();
}

void RlBackend::publishZeroTorqueCommand() {
  legged_msgs::msg::JointControlData msg;
  msg.joint_position.assign(kJointCount, 0.0);
  msg.joint_velocity.assign(kJointCount, 0.0);
  msg.joint_torque.assign(kJointCount, 0.0);
  msg.kp = 0.0;
  msg.kd = 0.0;
  jointControlPublisher_->publish(msg);
  publishEmergencyOverrideState();
}

void RlBackend::publishEmergencyOverrideState() const {
  if (!emergencyOverrideStatePublisher_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = static_cast<int>(controlState_);
  emergencyOverrideStatePublisher_->publish(msg);
}

ocs2::vector_t RlBackend::poseForControlState() const {
  switch (controlState_) {
    case ControlState::Hold:
      return hasHoldPose_ ? holdJointState_ : standJointState_;
    case ControlState::RecoveryPose:
      return recoveryJointState_;
    case ControlState::SitDown:
      return sitJointState_;
    case ControlState::ZeroTorque:
      return ocs2::vector_t::Zero(kJointCount);
    case ControlState::Stand:
    case ControlState::Policy:
      return standJointState_;
  }

  return standJointState_;
}

geometry_msgs::msg::Twist RlBackend::activeVelocityCommand() const {
  if (lastCommandReceiptTime_.nanoseconds() == 0) {
    return geometry_msgs::msg::Twist{};
  }

  const double age = (node_->now() - lastCommandReceiptTime_).seconds();
  if (!std::isfinite(age) || age > settings_.commandTimeoutSec) {
    return geometry_msgs::msg::Twist{};
  }

  return latestVelocityCommand_;
}

geometry_msgs::msg::Twist RlBackend::policyVelocityCommand() const {
  geometry_msgs::msg::Twist command = activeVelocityCommand();
  command.linear.x *= commandObservationScale_(0);
  command.linear.y *= commandObservationScale_(1);
  command.angular.z *= commandObservationScale_(2);
  return command;
}
