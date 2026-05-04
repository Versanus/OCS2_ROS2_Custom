#include "motion_control/controller/RlBackend.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

#include "motion_control/controller/OrtCpuRunner.h"
#include "motion_control/legged_estimation/StateEstimatorFactory.h"

namespace {

constexpr std::size_t kJointCount = 12;
constexpr std::array<const char*, kJointCount> kInternalJointNames = {
    "LF_HAA", "LF_HFE", "LF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
};

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

template <typename T>
void writeJsonArray(std::ostream& stream, const std::vector<T>& values) {
  stream << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << "]";
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

std::string compactLowercase(std::string value) {
  value.erase(
      std::remove_if(value.begin(), value.end(), [](unsigned char ch) {
        return ch == '_' || ch == '-' || std::isspace(ch);
      }),
      value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::vector<std::string> stringListFromInfo(const boost::property_tree::ptree& tree, const std::string& path) {
  std::vector<std::string> values;
  if (const auto child = tree.get_child_optional(path)) {
    for (const auto& entry : *child) {
      const std::string value = entry.second.get_value<std::string>("");
      if (!value.empty()) {
        values.push_back(value);
      }
    }
  }

  if (values.empty()) {
    if (const auto flatValue = tree.get_optional<std::string>(path)) {
      std::string normalized = *flatValue;
      std::replace(normalized.begin(), normalized.end(), ',', ' ');
      std::istringstream stream(normalized);
      std::string token;
      while (stream >> token) {
        values.push_back(token);
      }
    }
  }

  return values;
}

std::size_t jointIndexFromName(const std::string& name) {
  const std::string token = compactLowercase(name);
  for (std::size_t i = 0; i < kInternalJointNames.size(); ++i) {
    if (token == compactLowercase(kInternalJointNames[i])) {
      return i;
    }
  }

  throw std::runtime_error("Unknown joint name in policyActionJointOrder: " + name);
}

std::vector<std::size_t> jointOrderFromInfo(const boost::property_tree::ptree& tree, const std::string& path) {
  const std::vector<std::string> jointNames = stringListFromInfo(tree, path);
  std::vector<std::size_t> jointOrder;
  jointOrder.reserve(kJointCount);

  if (jointNames.empty()) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      jointOrder.push_back(i);
    }
    return jointOrder;
  }

  if (jointNames.size() != kJointCount) {
    throw std::runtime_error(path + " must list exactly 12 joints.");
  }

  std::array<bool, kJointCount> seen{};
  for (const auto& jointName : jointNames) {
    const std::size_t jointIndex = jointIndexFromName(jointName);
    if (seen[jointIndex]) {
      throw std::runtime_error(path + " contains a duplicate joint: " + jointName);
    }
    seen[jointIndex] = true;
    jointOrder.push_back(jointIndex);
  }

  return jointOrder;
}

bool isFiniteEigenVector(const ocs2::vector_t& values, std::size_t expectedSize) {
  if (values.size() != static_cast<Eigen::Index>(expectedSize)) {
    return false;
  }
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) {
      return false;
    }
  }
  return true;
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

RlBackend::ObservationLayout RlBackend::parseObservationLayout(const std::string& value) {
  const std::string token = compactLowercase(value);
  if (token == "legacy" || token == "current" || token == "training") {
    return ObservationLayout::Legacy;
  }
  if (token == "policy" || token == "joystick" || token == "onnx") {
    return ObservationLayout::Policy;
  }

  throw std::runtime_error("observationLayout must be 'legacy' or 'policy'.");
}

RlBackend::JointPositionObservation RlBackend::parseJointPositionObservation(const std::string& value) {
  const std::string token = compactLowercase(value);
  if (token == "relativetodefault" || token == "relative" || token == "qdefault" ||
      token == "qminusdefault") {
    return JointPositionObservation::RelativeToDefault;
  }
  if (token == "raw" || token == "absolute" || token == "q") {
    return JointPositionObservation::Raw;
  }

  throw std::runtime_error("jointPositionObservation must be 'relative_to_default' or 'raw'.");
}

RlBackend::FeedbackJointStateTransform RlBackend::parseFeedbackJointStateTransform(const std::string& value) {
  const std::string token = compactLowercase(value);
  if (token.empty() || token == "none" || token == "off" || token == "false") {
    return FeedbackJointStateTransform::None;
  }
  if (token == "quadminihardware" || token == "hardwaretopolicy" || token == "hardwaretomujoco") {
    return FeedbackJointStateTransform::QuadMiniHardware;
  }

  throw std::runtime_error("feedbackJointStateTransform must be 'none' or 'quad_mini_hardware'.");
}

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
      controlState_(ControlState::Stand),
      poseTransitionStartTime_(0, 0, RCL_ROS_TIME),
      poseTransitionDurationSec_(0.0),
      defaultJointState_(ocs2::vector_t::Zero(kJointCount)),
      holdJointState_(ocs2::vector_t::Zero(kJointCount)),
      poseTransitionStartPose_(ocs2::vector_t::Zero(kJointCount)),
      poseTransitionTargetPose_(ocs2::vector_t::Zero(kJointCount)),
      standJointState_(ocs2::vector_t::Zero(kJointCount)),
      recoveryJointState_(ocs2::vector_t::Zero(kJointCount)),
      sitJointState_(ocs2::vector_t::Zero(kJointCount)),
      lastAction_(kJointCount, 0.0f) {}

RlBackend::~RlBackend() = default;

bool RlBackend::configure(const ControllerConfig& config) {
  if (!node_) {
    return false;
  }
  if (config.rlConfigFile.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RL backend requires a non-empty rlConfigFile parameter.");
    return false;
  }

  controllerConfig_ = config;
  if (!loadSettings(config)) {
    return false;
  }
  if (!loadPoses(config.rlConfigFile)) {
    return false;
  }
  if (!setupStateEstimator()) {
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
  prepareDebugDumpDirectory();

  RCLCPP_INFO(node_->get_logger(),
              "Configured RL backend with model='%s' input='%s'(%zu) output='%s'(%zu) policy_hz=%.2f "
              "action_scale=%.3f kp_ratio=%.3f kd_ratio=%.3f pose_kp_ratio=%.3f pose_kd_ratio=%.3f.",
              settings_.modelPath.c_str(), policyRunner_->inputName().c_str(), policyRunner_->inputSize(),
              policyRunner_->outputName().c_str(), policyRunner_->outputSize(),
              settings_.policyHz, settings_.actionScale, settings_.kpRatio, settings_.kdRatio,
              settings_.poseKpRatio, settings_.poseKdRatio);
  return true;
}

void RlBackend::launch() {
  setupRosInterfaces();
  initializeStandState();
  RCLCPP_INFO(node_->get_logger(),
              "RL backend starts in rlStandJointState position hold. Enter mode:vel and press '2' to arm RL policy output.");

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, settings_.policyHz));
  policyTimer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RlBackend::policyTimerCallback, this));

  publishEmergencyOverrideState();
  publishPoseCommand(standJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
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
    estimatorConfig_ = loadEstimatorConfig(rlConfigPath.string(), false);

    settings_.modelPath = resolvePath(rlConfigPath, tree.get<std::string>("onnxModelPath")).string();
    settings_.inputName = tree.get<std::string>("onnxInputName", settings_.inputName);
    settings_.outputName = tree.get<std::string>("onnxOutputName", settings_.outputName);
    settings_.policyHz = tree.get<double>("policyHz", settings_.policyHz);
    settings_.actionScale = tree.get<double>("actionScale", settings_.actionScale);
    settings_.actionClip = tree.get<double>("actionClip", settings_.actionClip);
    settings_.actionRateLimit = tree.get<double>("actionRateLimit", settings_.actionRateLimit);
    settings_.kpRatio = tree.get<double>("kpRatio", settings_.kpRatio);
    settings_.kdRatio = tree.get<double>("kdRatio", settings_.kdRatio);
    settings_.poseKpRatio = tree.get<double>("poseKpRatio", settings_.poseKpRatio);
    settings_.poseKdRatio = tree.get<double>("poseKdRatio", settings_.poseKdRatio);
    settings_.stateTimeoutSec = tree.get<double>("stateTimeoutSec", settings_.stateTimeoutSec);
    settings_.commandTimeoutSec = tree.get<double>("commandTimeoutSec", settings_.commandTimeoutSec);
    settings_.startupHoldSec = tree.get<double>("startupHoldSec", settings_.startupHoldSec);
    settings_.standTransitionSec = tree.get<double>("standTransitionSec", settings_.standTransitionSec);
    settings_.sitTransitionSec = tree.get<double>("sitTransitionSec", settings_.sitTransitionSec);
    settings_.commandActivationThreshold =
        tree.get<double>("commandActivationThreshold", settings_.commandActivationThreshold);
    settings_.velocityCommandCap =
        tree.get<double>("velocityCommandCap", settings_.velocityCommandCap);
    settings_.invertVelocityCommandXY =
        tree.get<bool>("invertVelocityCommandXY", settings_.invertVelocityCommandXY);
    settings_.requireCommandForPolicy =
        tree.get<bool>("requireCommandForPolicy", settings_.requireCommandForPolicy);
    settings_.holdStandWhenPolicyIdle =
        tree.get<bool>("holdStandWhenPolicyIdle", settings_.holdStandWhenPolicyIdle);
    settings_.observationDim = static_cast<std::size_t>(tree.get<int>("obsDim", static_cast<int>(settings_.observationDim)));
    settings_.actionDim = static_cast<std::size_t>(tree.get<int>("actDim", static_cast<int>(settings_.actionDim)));
    settings_.observationLayout =
        parseObservationLayout(tree.get<std::string>("observationLayout", "legacy"));
    const std::string defaultJointPositionObservation =
        settings_.observationLayout == ObservationLayout::Policy ? "raw" : "relative_to_default";
    settings_.jointPositionObservation = parseJointPositionObservation(
        tree.get<std::string>("jointPositionObservation", defaultJointPositionObservation));
    const std::string feedbackJointStateTransform =
        config.rlFeedbackJointStateTransform.empty()
            ? tree.get<std::string>("feedbackJointStateTransform", "none")
            : config.rlFeedbackJointStateTransform;
    settings_.feedbackJointStateTransform = parseFeedbackJointStateTransform(
        feedbackJointStateTransform);
    if (!config.rlFeedbackJointStateTransform.empty()) {
      RCLCPP_INFO(node_->get_logger(), "Overriding RL feedback joint-state transform with '%s'.",
                  feedbackJointStateTransform.c_str());
    }
    settings_.actionToJointIndex = jointOrderFromInfo(tree, "policyActionJointOrder");
    settings_.observationToJointIndex = tree.get_child_optional("policyJointObservationOrder")
                                            ? jointOrderFromInfo(tree, "policyJointObservationOrder")
                                            : settings_.actionToJointIndex;
    settings_.debugDumpEnabled = tree.get<bool>("debugDumpEnabled", settings_.debugDumpEnabled);
    settings_.debugDumpDir = tree.get<std::string>("debugDumpDir", settings_.debugDumpDir);
    settings_.debugDumpMaxSteps =
        static_cast<std::size_t>(tree.get<int>("debugDumpMaxSteps", static_cast<int>(settings_.debugDumpMaxSteps)));

    if (settings_.modelPath.empty()) {
      throw std::runtime_error("onnxModelPath must not be empty.");
    }
    if (!fs::exists(settings_.modelPath)) {
      throw std::runtime_error("Resolved ONNX model path does not exist: " + settings_.modelPath);
    }
    if (settings_.observationDim == 0 || settings_.actionDim == 0) {
      throw std::runtime_error("obsDim and actDim must be positive.");
    }
    if (settings_.actionDim != kJointCount) {
      throw std::runtime_error("The RL backend currently requires actDim to be 12.");
    }
    if (settings_.actionToJointIndex.size() != settings_.actionDim) {
      throw std::runtime_error("policyActionJointOrder size must match actDim.");
    }
    if (settings_.observationToJointIndex.size() != kJointCount) {
      throw std::runtime_error("policyJointObservationOrder must contain 12 joints.");
    }
    if (!std::isfinite(settings_.actionScale) || !std::isfinite(settings_.actionClip) ||
        !std::isfinite(settings_.actionRateLimit) || settings_.actionClip < 0.0 || settings_.actionRateLimit < 0.0) {
      throw std::runtime_error("actionScale, actionClip, and actionRateLimit must be finite and non-negative.");
    }
    if (!std::isfinite(settings_.kpRatio) || !std::isfinite(settings_.kdRatio) ||
        !std::isfinite(settings_.poseKpRatio) || !std::isfinite(settings_.poseKdRatio) ||
        settings_.kpRatio < 0.0 || settings_.kdRatio < 0.0 ||
        settings_.poseKpRatio < 0.0 || settings_.poseKdRatio < 0.0) {
      throw std::runtime_error("kpRatio/kdRatio and poseKpRatio/poseKdRatio must be finite and non-negative.");
    }
    if (!std::isfinite(settings_.standTransitionSec) || settings_.standTransitionSec < 0.0) {
      throw std::runtime_error("standTransitionSec must be finite and non-negative.");
    }
    if (!std::isfinite(settings_.sitTransitionSec) || settings_.sitTransitionSec < 0.0) {
      throw std::runtime_error("sitTransitionSec must be finite and non-negative.");
    }
    if (!std::isfinite(settings_.velocityCommandCap) || settings_.velocityCommandCap < 0.0) {
      throw std::runtime_error("velocityCommandCap must be finite and non-negative.");
    }
    if (settings_.debugDumpEnabled && settings_.debugDumpDir.empty()) {
      settings_.debugDumpDir = "/tmp/quad_mini_rl_debug";
    }
    RCLCPP_INFO(node_->get_logger(), "Configured RL state estimator: enabled=%s type=%s orientationSource=%s",
                estimatorConfig_.enabled ? "true" : "false", toString(estimatorConfig_.type),
                toString(estimatorConfig_.orientationSource));
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load RL config '%s': %s", config.rlConfigFile.c_str(), error.what());
    return false;
  }
}

bool RlBackend::setupStateEstimator() {
  hasEstimatedState_ = false;
  estimatorSeeded_ = false;
  lastEstimatedStateReceiptTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lastEstimatedStateSimTime_ = 0.0;
  stateEstimator_.reset();
  eeKinematicsPtr_.reset();
  leggedInterface_.reset();

  if (!estimatorConfig_.enabled) {
    return true;
  }

  try {
    leggedInterface_ = std::make_shared<LeggedRobotInterface>(controllerConfig_.taskFile, controllerConfig_.urdfFile,
                                                              controllerConfig_.referenceFile);
    ocs2::CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
    eeKinematicsPtr_ = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(
        leggedInterface_->getPinocchioInterface(), pinocchioMapping, leggedInterface_->modelSettings().contactNames3DoF);
    eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());
    stateEstimator_ = StateEstimatorFactory::create(estimatorConfig_, node_, leggedInterface_->getPinocchioInterface(),
                                                    leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_,
                                                    controllerConfig_.taskFile, false);
    estimatedRbdState_.setZero(2 * leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum);
    estimatedOrientation_ = quaternion_t::Identity();
    estimatedLocalLinearVelocity_.setZero();
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to configure RL state estimator: %s", error.what());
    return false;
  }
}

bool RlBackend::loadPoses(const std::string& rlConfigFile) {
  try {
    boost::property_tree::ptree rlTree;
    boost::property_tree::read_info(rlConfigFile, rlTree);

    ocs2::loadData::loadEigenMatrix(rlConfigFile, "rlStandJointState", standJointState_);
    ocs2::loadData::loadEigenMatrix(rlConfigFile, "rlRecoveryJointState", recoveryJointState_);
    ocs2::loadData::loadEigenMatrix(rlConfigFile, "rlSitJointState", sitJointState_);
    defaultJointState_ = standJointState_;

    if (rlTree.get_child_optional("policyDefaultJointState")) {
      ocs2::vector_t policyDefaultJointState = ocs2::vector_t::Zero(kJointCount);
      ocs2::loadData::loadEigenMatrix(rlConfigFile, "policyDefaultJointState", policyDefaultJointState);
      if (!isFiniteEigenVector(policyDefaultJointState, kJointCount)) {
        throw std::runtime_error("policyDefaultJointState in rl.info must contain 12 finite values.");
      }
      defaultJointState_ = policyDefaultJointState;
      RCLCPP_INFO(node_->get_logger(),
                  "Using policyDefaultJointState from rl.info for RL q_default.");
    }

    if (!isFiniteEigenVector(defaultJointState_, kJointCount) ||
        !isFiniteEigenVector(standJointState_, kJointCount) ||
        !isFiniteEigenVector(recoveryJointState_, kJointCount) ||
        !isFiniteEigenVector(sitJointState_, kJointCount)) {
      throw std::runtime_error("RL pose blocks must contain 12 finite joint values.");
    }

    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load RL poses from '%s': %s",
                 rlConfigFile.c_str(), error.what());
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

void RlBackend::initializeStandState() {
  controlState_ = ControlState::Stand;
  startupHoldStarted_ = false;
  holdJointState_ = standJointState_;
  hasHoldPose_ = true;
  resetPoseTransition();
  lastAction_.assign(settings_.actionDim, 0.0f);
}

void RlBackend::stateCallback(const legged_msgs::msg::SimulatorStateData::SharedPtr msg) {
  latestState_ = *msg;
  hasState_ = true;
  lastStateReceiptTime_ = node_->now();
  trySeedStateEstimatorFromLatestState();

  if (controlState_ == ControlState::Hold && !hasHoldPose_) {
    latchHoldPoseFromLatestState();
  }
}

void RlBackend::sensorCallback(const legged_msgs::msg::SimulatorSensorData::SharedPtr msg) {
  latestSensor_ = *msg;
  hasSensor_ = true;
  lastSensorReceiptTime_ = node_->now();
  trySeedStateEstimatorFromLatestState();
  if (stateEstimator_) {
    updateStateEstimatorFromSensor(*msg);
  }
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
      hasHoldPose_ = true;
      holdJointState_ = standJointState_;
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
                  "RL backend switched to sit pose. Sitting down over %.2f s with PD gain ratios kp=%.3f kd=%.3f. "
                  "start=[%s] target=[%s]",
                  settings_.sitTransitionSec, settings_.poseKpRatio, settings_.poseKdRatio,
                  poseSummary(poseTransitionStartPose_).c_str(), poseSummary(poseTransitionTargetPose_).c_str());
      publishPoseCommand(transitionPoseCommand(), settings_.poseKpRatio, settings_.poseKdRatio);
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
                  "RL backend switched to rlStandJointState hold over %.2f s with PD gain ratios kp=%.3f kd=%.3f. "
                  "start=[%s] target=[%s]",
                  settings_.standTransitionSec, settings_.poseKpRatio, settings_.poseKdRatio,
                  poseSummary(poseTransitionStartPose_).c_str(), poseSummary(poseTransitionTargetPose_).c_str());
      publishPoseCommand(transitionPoseCommand(), settings_.poseKpRatio, settings_.poseKdRatio);
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
    if (!hasHoldPose_ && hasFreshState()) {
      latchHoldPoseFromLatestState();
    }
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    }
    return;
  }

  if (controlState_ == ControlState::Stand) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (poseTransitionStartTime_.nanoseconds() == 0) {
      beginPoseTransition(standJointState_, settings_.standTransitionSec);
    }
    publishPoseCommand(transitionPoseCommand(), settings_.poseKpRatio, settings_.poseKdRatio);
    return;
  }

  if (controlState_ == ControlState::SitDown) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (poseTransitionStartTime_.nanoseconds() == 0) {
      beginPoseTransition(sitJointState_, settings_.sitTransitionSec);
    }
    publishPoseCommand(transitionPoseCommand(), settings_.poseKpRatio, settings_.poseKdRatio);
    return;
  }

  if (controlState_ != ControlState::Policy) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    publishPoseCommand(poseForControlState(), settings_.poseKpRatio, settings_.poseKdRatio);
    return;
  }

  if (!hasFreshState() || !hasFreshSensor()) {
    startupHoldStarted_ = false;
    lastAction_.assign(settings_.actionDim, 0.0f);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "RL backend lost fresh simulator state/sensor data. Holding rlStandJointState.");
    publishPoseCommand(standJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    return;
  }

  const auto now = node_->now();
  if (!startupHoldStarted_) {
    startupHoldStarted_ = true;
    firstFreshStateTime_ = now;
    hasHoldPose_ = true;
    holdJointState_ = standJointState_;
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (settings_.startupHoldSec > 0.0) {
      RCLCPP_INFO(node_->get_logger(),
                  "Fresh simulator state and sensor data received. Holding rlStandJointState for %.2f s before allowing RL policy output.",
                  settings_.startupHoldSec);
    } else {
      RCLCPP_INFO(node_->get_logger(), "Fresh simulator state and sensor data received. RL policy output enabled.");
    }
  }

  if (settings_.startupHoldSec > 0.0 && (now - firstFreshStateTime_).seconds() < settings_.startupHoldSec) {
    publishPoseCommand(standJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    return;
  }

  if (settings_.requireCommandForPolicy && !hasActiveVelocityCommand()) {
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (settings_.holdStandWhenPolicyIdle) {
      publishPoseCommand(standJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    } else {
      latchHoldPoseFromLatestState();
      if (hasHoldPose_) {
        publishPoseCommand(holdJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
      }
    }
    return;
  }

  hasHoldPose_ = true;
  holdJointState_ = standJointState_;

  const auto computeStart = std::chrono::steady_clock::now();
  ObservationSnapshot observationSnapshot;
  if (!buildObservationSnapshot(observationSnapshot)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to build RL observation. Holding the last latched pose.");
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    } else {
      publishPoseCommand(standJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    }
    return;
  }

  const auto inferStart = std::chrono::steady_clock::now();
  std::vector<float> action;
  if (!policyRunner_->infer(observationSnapshot.observation, action) || action.size() != settings_.actionDim) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "RL inference failed or returned an unexpected action size. Holding the last latched pose.");
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    } else {
      publishPoseCommand(standJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    }
    return;
  }
  if (!std::all_of(action.begin(), action.end(), [](float value) { return std::isfinite(value); })) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "RL inference produced non-finite actions. Holding the last latched pose.");
    lastAction_.assign(settings_.actionDim, 0.0f);
    if (hasHoldPose_) {
      publishPoseCommand(holdJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
    } else {
      publishPoseCommand(standJointState_, settings_.poseKpRatio, settings_.poseKdRatio);
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

  const PolicyCommandSnapshot commandSnapshot = buildPolicyCommandSnapshot(action);
  dumpPolicyStep(observationSnapshot, commandSnapshot);
  publishPolicyCommand(commandSnapshot);
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
  const bool requiresQuaternion = !stateEstimator_ || estimatorConfig_.orientationSource == OrientationSource::ImuQuaternion;
  const bool hasLinearVelocityObservation = stateEstimator_ ? hasFreshEstimatedState()
                                                            : (isFiniteVector(latestSensor_.local_linvel_values, 3) || hasFreshState());
  return std::isfinite(age) && age <= settings_.stateTimeoutSec &&
         (!requiresQuaternion || isFiniteVector(latestSensor_.imu_quat_values, 4)) &&
         isFiniteVector(latestSensor_.imu_angvel_values, 3) &&
         hasLinearVelocityObservation &&
         isFiniteVector(latestSensor_.joint_position_values, kJointCount) &&
         isFiniteVector(latestSensor_.joint_velocity_values, kJointCount);
}

bool RlBackend::hasFreshEstimatedState() const {
  if (!stateEstimator_ || !hasEstimatedState_) {
    return false;
  }

  const auto age = (node_->now() - lastEstimatedStateReceiptTime_).seconds();
  return std::isfinite(age) && age <= settings_.stateTimeoutSec &&
         estimatedRbdState_.size() > 0 &&
         estimatedOrientation_.norm() > 1e-6 &&
         estimatedLocalLinearVelocity_.allFinite();
}

bool RlBackend::buildRbdStateFromStateMsg(const legged_msgs::msg::SimulatorStateData& msg, ocs2::vector_t& rbdState) const {
  if (!leggedInterface_ && !hasFreshState()) {
    return false;
  }

  const std::size_t expectedJointCount = leggedInterface_ ? leggedInterface_->getCentroidalModelInfo().actuatedDofNum : kJointCount;
  if (!isFiniteVector(msg.base_quat_values, 4) || !isFiniteVector(msg.base_pose_values, 3) ||
      !isFiniteVector(msg.base_angvel_values, 3) || !isFiniteVector(msg.base_linvel_values, 3) ||
      !isFiniteVector(msg.joint_position_values, expectedJointCount) || !isFiniteVector(msg.joint_velocity_values, expectedJointCount)) {
    return false;
  }

  const Eigen::Quaterniond quaternion = normalizedQuaternionFromState(msg);
  const vector3_t eulerAngles = quatToZyx(quaternion.cast<ocs2::scalar_t>());
  const vector3_t angularVelocity(msg.base_angvel_values[0], msg.base_angvel_values[1], msg.base_angvel_values[2]);
  const vector3_t position(msg.base_pose_values[0], msg.base_pose_values[1], msg.base_pose_values[2]);
  const vector3_t linearVelocity(msg.base_linvel_values[0], msg.base_linvel_values[1], msg.base_linvel_values[2]);

  rbdState = ocs2::vector_t::Zero(2 * (6 + static_cast<Eigen::Index>(expectedJointCount)));
  rbdState.segment<3>(0) = eulerAngles;
  rbdState.segment<3>(3) = position;
  for (std::size_t i = 0; i < expectedJointCount; ++i) {
    rbdState(6 + static_cast<Eigen::Index>(i)) = msg.joint_position_values[i];
    rbdState(12 + static_cast<Eigen::Index>(expectedJointCount) + static_cast<Eigen::Index>(i)) = msg.joint_velocity_values[i];
  }
  rbdState.segment<3>(6 + static_cast<Eigen::Index>(expectedJointCount)) = angularVelocity;
  rbdState.segment<3>(9 + static_cast<Eigen::Index>(expectedJointCount)) = linearVelocity;
  return true;
}

bool RlBackend::trySeedStateEstimatorFromLatestState() {
  if (!stateEstimator_ || estimatorSeeded_ || !hasFreshState()) {
    return false;
  }

  ocs2::vector_t seedRbdState;
  if (!buildRbdStateFromStateMsg(latestState_, seedRbdState)) {
    return false;
  }

  stateEstimator_->seed(seedRbdState);
  estimatorSeeded_ = true;
  return true;
}

void RlBackend::updateStateEstimatorFromSensor(const legged_msgs::msg::SimulatorSensorData& msg) {
  if (!stateEstimator_) {
    return;
  }

  const std::size_t expectedJointCount = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;
  if (!isFiniteVector(msg.imu_angvel_values, 3) || !isFiniteVector(msg.imu_linacc_values, 3) ||
      !isFiniteVector(msg.joint_position_values, expectedJointCount) || !isFiniteVector(msg.joint_velocity_values, expectedJointCount)) {
    return;
  }

  quaternion_t quaternion = quaternion_t::Identity();
  if (isFiniteVector(msg.imu_quat_values, 4)) {
    quaternion = quaternion_t(msg.imu_quat_values[0], msg.imu_quat_values[1], msg.imu_quat_values[2], msg.imu_quat_values[3]);
    if (quaternion.norm() > 1e-6) {
      quaternion.normalize();
    } else {
      quaternion = quaternion_t::Identity();
    }
  } else if (estimatorConfig_.orientationSource == OrientationSource::ImuQuaternion) {
    return;
  }

  contact_flag_t contactFlags{};
  contactFlags.fill(false);
  for (std::size_t i = 0; i < std::min<std::size_t>(contactFlags.size(), msg.contact_flags.size()); ++i) {
    contactFlags[i] = msg.contact_flags[i];
  }

  ocs2::vector_t jointPositions(expectedJointCount);
  ocs2::vector_t jointVelocities(expectedJointCount);
  for (std::size_t i = 0; i < expectedJointCount; ++i) {
    jointPositions(static_cast<Eigen::Index>(i)) = msg.joint_position_values[i];
    jointVelocities(static_cast<Eigen::Index>(i)) = msg.joint_velocity_values[i];
  }

  const vector3_t angularVelocity(msg.imu_angvel_values[0], msg.imu_angvel_values[1], msg.imu_angvel_values[2]);
  const vector3_t linearAcceleration(msg.imu_linacc_values[0], msg.imu_linacc_values[1], msg.imu_linacc_values[2]);
  const matrix3_t zeroCovariance = matrix3_t::Zero();

  stateEstimator_->updateJointStates(jointPositions, jointVelocities);
  stateEstimator_->updateContact(contactFlags);
  stateEstimator_->updateImu(quaternion, angularVelocity, linearAcceleration, zeroCovariance, zeroCovariance, zeroCovariance);

  const ocs2::scalar_t updateTime = static_cast<ocs2::scalar_t>(msg.simulation_time);
  ocs2::scalar_t updatePeriod = 0.0;
  if (hasEstimatedState_) {
    const ocs2::scalar_t previousTime = lastEstimatedStateSimTime_;
    updatePeriod = updateTime - previousTime;
    if (updatePeriod < 0.0) {
      stateEstimator_->reset();
      estimatorSeeded_ = false;
      trySeedStateEstimatorFromLatestState();
      updatePeriod = 0.0;
    }
  }

  estimatedRbdState_ = stateEstimator_->update(updateTime, updatePeriod);
  estimatedOrientation_ = stateEstimator_->getOrientationQuaternion();
  estimatedLocalLinearVelocity_ = stateEstimator_->getBaseLinearVelocityLocal();
  hasEstimatedState_ = true;
  lastEstimatedStateReceiptTime_ = node_->now();
  lastEstimatedStateSimTime_ = updateTime;
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

std::string RlBackend::poseSummary(const ocs2::vector_t& pose) const {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << kInternalJointNames[i] << "=" << pose[static_cast<Eigen::Index>(i)];
  }
  return stream.str();
}

bool RlBackend::buildObservationSnapshot(ObservationSnapshot& snapshot) const {
  if (!hasFreshSensor()) {
    return false;
  }

  try {
    std::vector<double> jointPositions = latestSensor_.joint_position_values;
    std::vector<double> jointVelocities = latestSensor_.joint_velocity_values;
    transformFeedbackJointState(jointPositions, jointVelocities);

    const Eigen::Quaterniond quaternion =
        stateEstimator_ ? estimatedOrientation_.cast<double>() : normalizedQuaternionFromSensor(latestSensor_);
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const Eigen::Vector3d projectedGravity = rotation.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
    Eigen::Vector3d baseLinearVelocity = Eigen::Vector3d::Zero();
    if (stateEstimator_) {
      baseLinearVelocity = estimatedLocalLinearVelocity_.cast<double>();
    } else if (isFiniteVector(latestSensor_.local_linvel_values, 3)) {
      baseLinearVelocity << latestSensor_.local_linvel_values[0], latestSensor_.local_linvel_values[1],
          latestSensor_.local_linvel_values[2];
    } else if (hasFreshState()) {
      baseLinearVelocity << latestState_.base_linvel_values[0], latestState_.base_linvel_values[1],
          latestState_.base_linvel_values[2];
    } else {
      return false;
    }
    const geometry_msgs::msg::Twist velocityCommand = policyVelocityCommand();

    snapshot = ObservationSnapshot{};
    snapshot.sensorTime = latestSensor_.simulation_time;
    snapshot.imuQuaternion = {
        static_cast<float>(quaternion.w()),
        static_cast<float>(quaternion.x()),
        static_cast<float>(quaternion.y()),
        static_cast<float>(quaternion.z()),
    };
    snapshot.observation.clear();
    snapshot.observation.reserve(settings_.observationDim);

    const auto appendProjectedGravity = [&]() {
      for (int i = 0; i < 3; ++i) {
        const float value = static_cast<float>(projectedGravity[i]);
        snapshot.projectedGravity.push_back(value);
        snapshot.observation.push_back(value);
      }
    };
    const auto appendBaseLinearVelocity = [&]() {
      for (int i = 0; i < 3; ++i) {
        const float value = static_cast<float>(baseLinearVelocity[i]);
        snapshot.baseLinearVelocity.push_back(value);
        snapshot.observation.push_back(value);
      }
    };
    const auto appendBaseAngularVelocity = [&]() {
      for (int i = 0; i < 3; ++i) {
        const float value = static_cast<float>(latestSensor_.imu_angvel_values[i]);
        snapshot.baseAngularVelocity.push_back(value);
        snapshot.observation.push_back(value);
      }
    };

    if (settings_.observationLayout == ObservationLayout::Policy) {
      appendProjectedGravity();
      appendBaseLinearVelocity();
      appendBaseAngularVelocity();
    } else {
      appendBaseLinearVelocity();
      appendBaseAngularVelocity();
      appendProjectedGravity();
    }

    for (std::size_t observationIndex = 0; observationIndex < settings_.observationToJointIndex.size(); ++observationIndex) {
      const std::size_t jointIndex = settings_.observationToJointIndex[observationIndex];
      const double jointPositionRaw = jointPositions[jointIndex];
      const float jointPositionRelativeToDefault = static_cast<float>(
          jointPositionRaw - defaultJointState_[static_cast<Eigen::Index>(jointIndex)]);
      const float jointPosition =
          settings_.jointPositionObservation == JointPositionObservation::Raw
              ? static_cast<float>(jointPositionRaw)
              : jointPositionRelativeToDefault;
      snapshot.jointPositionRelativeToDefault.push_back(jointPositionRelativeToDefault);
      snapshot.observation.push_back(jointPosition);
    }
    for (std::size_t observationIndex = 0; observationIndex < settings_.observationToJointIndex.size(); ++observationIndex) {
      const std::size_t jointIndex = settings_.observationToJointIndex[observationIndex];
      const float jointVelocity = static_cast<float>(jointVelocities[jointIndex]);
      snapshot.jointVelocity.push_back(jointVelocity);
      snapshot.observation.push_back(jointVelocity);
    }
    snapshot.previousAction = lastAction_;
    snapshot.observation.insert(snapshot.observation.end(), lastAction_.begin(), lastAction_.end());
    snapshot.command = {
        static_cast<float>(velocityCommand.linear.x),
        static_cast<float>(velocityCommand.linear.y),
        static_cast<float>(velocityCommand.angular.z),
    };
    snapshot.observation.insert(snapshot.observation.end(), snapshot.command.begin(), snapshot.command.end());

    if (snapshot.observation.size() != settings_.observationDim) {
      return false;
    }

    return std::all_of(snapshot.observation.begin(), snapshot.observation.end(), [](float value) { return std::isfinite(value); });
  } catch (...) {
    return false;
  }
}

void RlBackend::transformFeedbackJointState(std::vector<double>& jointPositions,
                                            std::vector<double>& jointVelocities) const {
  if (settings_.feedbackJointStateTransform != FeedbackJointStateTransform::QuadMiniHardware) {
    return;
  }
  if (jointPositions.size() < kJointCount || jointVelocities.size() < kJointCount) {
    return;
  }

  struct JointFeedbackTransform {
    double referenceOffset;
    double homeOffset;
    double calibrationOffset;
    double sign;
    bool knee;
    bool referenceUsesPositiveHomeOffset;
  };

  static constexpr std::array<JointFeedbackTransform, kJointCount> kQuadMiniHardwareFeedbackTransform = {{
      {-0.322415828704834,    0.322415828704834,   0.0,                 -1.0, false, true},   // LF_HAA / FL_HipX
      { 0.4309038817882538,   0.2509038817882538, -1.050324412,          1.0, false, true},   // LF_HFE / FL_HipY
      {-0.4077022075653076,  -0.4077022075653076,  1.9584364492350666,   1.0, true,  true},   // LF_KFE / FL_Knee
      {-0.32241925597190857,  0.32241925597190857, 0.0,                  1.0, false, false},  // LH_HAA / HL_HipX
      { 0.43023791909217834,  0.25023791909217834,-0.950324412,          1.0, false, true},   // LH_HFE / HL_HipY
      {-0.4072657823562622,  -0.4072657823562622,  1.9584364492350666,   1.0, true,  true},   // LH_KFE / HL_Knee
      {-0.33759668469429016, -0.33759668469429016, 0.0,                 -1.0, false, false},  // RF_HAA / FR_HipX
      { 0.44108203053474426,  0.25108203053474426, 1.050324412,         -1.0, false, false},  // RF_HFE / FR_HipY
      { 0.42747020721435547, -0.42747020721435547,-1.9558436449235067,  -1.0, true,  false},  // RF_KFE / FR_Knee
      {-0.3375966548919678,  -0.3375966548919678,  0.0,                  1.0, false, true},   // RH_HAA / HR_HipX
      { 0.44040337204933167,  0.25040337204933167, 0.950324412,         -1.0, false, false},  // RH_HFE / HR_HipY
      { 0.4270230531692505,  -0.4270230531692505, -1.9584364492350666,  -1.0, true,  false},  // RH_KFE / HR_Knee
  }};

  constexpr double kKneeDeviationScale = 1.5;
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto& transform = kQuadMiniHardwareFeedbackTransform[i];
    const double signedHomeOffset =
        transform.sign > 0.0 ? transform.homeOffset : -transform.homeOffset;
    const double commandOffset = transform.calibrationOffset + signedHomeOffset;
    double unscaledHardwarePosition = jointPositions[i];
    double velocityScale = transform.sign;

    if (transform.knee) {
      const double referenceHomeOffset =
          transform.referenceUsesPositiveHomeOffset ? transform.homeOffset : -transform.homeOffset;
      const double hardwareReference =
          transform.referenceOffset + transform.calibrationOffset + referenceHomeOffset;
      unscaledHardwarePosition =
          hardwareReference + (jointPositions[i] - hardwareReference) / kKneeDeviationScale;
      velocityScale *= kKneeDeviationScale;
    }

    jointPositions[i] = (unscaledHardwarePosition - commandOffset) / transform.sign;
    jointVelocities[i] = jointVelocities[i] / velocityScale;
  }
}

RlBackend::PolicyCommandSnapshot RlBackend::buildPolicyCommandSnapshot(const std::vector<float>& action) const {
  PolicyCommandSnapshot snapshot;
  snapshot.actionRaw = action;
  snapshot.actionClipped.resize(settings_.actionDim, 0.0f);
  snapshot.jointTarget.resize(kJointCount, 0.0f);

  for (std::size_t i = 0; i < kJointCount; ++i) {
    snapshot.jointTarget[i] = static_cast<float>(defaultJointState_[static_cast<Eigen::Index>(i)]);
  }

  for (std::size_t actionIndex = 0; actionIndex < settings_.actionToJointIndex.size(); ++actionIndex) {
    const std::size_t jointIndex = settings_.actionToJointIndex[actionIndex];
    double clippedAction = clampMagnitude(static_cast<double>(action[actionIndex]), settings_.actionClip);
    if (settings_.actionRateLimit > 0.0 && actionIndex < lastAction_.size()) {
      const double previousAction = static_cast<double>(lastAction_[actionIndex]);
      clippedAction = std::clamp(clippedAction,
                                 previousAction - settings_.actionRateLimit,
                                 previousAction + settings_.actionRateLimit);
    }
    snapshot.actionClipped[actionIndex] = static_cast<float>(clippedAction);
    snapshot.jointTarget[jointIndex] = static_cast<float>(
        defaultJointState_[static_cast<Eigen::Index>(jointIndex)] + settings_.actionScale * clippedAction);
  }

  return snapshot;
}

void RlBackend::prepareDebugDumpDirectory() {
  debugDumpDir_.clear();
  debugDumpedPolicySteps_ = 0;

  if (!settings_.debugDumpEnabled) {
    return;
  }

  try {
    debugDumpDir_ = std::filesystem::path(settings_.debugDumpDir) / "controller";
    std::filesystem::create_directories(debugDumpDir_);
    RCLCPP_INFO(node_->get_logger(), "RL debug dump enabled. Writing policy snapshots to '%s'.",
                debugDumpDir_.string().c_str());
  } catch (const std::exception& error) {
    debugDumpDir_.clear();
    RCLCPP_ERROR(node_->get_logger(), "Failed to prepare RL debug dump directory '%s': %s",
                 settings_.debugDumpDir.c_str(), error.what());
  }
}

void RlBackend::dumpPolicyStep(const ObservationSnapshot& observationSnapshot,
                               const PolicyCommandSnapshot& commandSnapshot) const {
  if (!settings_.debugDumpEnabled || debugDumpDir_.empty()) {
    return;
  }
  if (settings_.debugDumpMaxSteps > 0 && debugDumpedPolicySteps_ >= settings_.debugDumpMaxSteps) {
    return;
  }

  try {
    const auto filePath = debugDumpDir_ /
        ("policy_step_" + [&]() {
          std::ostringstream name;
          name << std::setw(6) << std::setfill('0') << debugDumpedPolicySteps_;
          return name.str();
        }() + ".json");
    std::ofstream stream(filePath);
    stream.setf(std::ios::fixed);
    stream.precision(9);
    stream << "{\n";
    stream << "  \"policy_step\": " << debugDumpedPolicySteps_ << ",\n";
    stream << "  \"sensor_time\": " << observationSnapshot.sensorTime << ",\n";
    stream << "  \"imu_quat\": ";
    writeJsonArray(stream, observationSnapshot.imuQuaternion);
    stream << ",\n  \"base_lin_vel\": ";
    writeJsonArray(stream, observationSnapshot.baseLinearVelocity);
    stream << ",\n  \"base_ang_vel\": ";
    writeJsonArray(stream, observationSnapshot.baseAngularVelocity);
    stream << ",\n  \"projected_gravity\": ";
    writeJsonArray(stream, observationSnapshot.projectedGravity);
    stream << ",\n  \"joint_pos_rel_default\": ";
    writeJsonArray(stream, observationSnapshot.jointPositionRelativeToDefault);
    stream << ",\n  \"joint_vel\": ";
    writeJsonArray(stream, observationSnapshot.jointVelocity);
    stream << ",\n  \"previous_action\": ";
    writeJsonArray(stream, observationSnapshot.previousAction);
    stream << ",\n  \"command\": ";
    writeJsonArray(stream, observationSnapshot.command);
    stream << ",\n  \"onnx_input\": ";
    writeJsonArray(stream, observationSnapshot.observation);
    stream << ",\n  \"onnx_output_raw\": ";
    writeJsonArray(stream, commandSnapshot.actionRaw);
    stream << ",\n  \"action_clipped\": ";
    writeJsonArray(stream, commandSnapshot.actionClipped);
    stream << ",\n  \"q_des\": ";
    writeJsonArray(stream, commandSnapshot.jointTarget);
    stream << "\n}\n";
  } catch (const std::exception& error) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Failed to write RL debug snapshot: %s", error.what());
  }
}

void RlBackend::publishPolicyCommand(const std::vector<float>& action) {
  publishPolicyCommand(buildPolicyCommandSnapshot(action));
}

void RlBackend::publishPolicyCommand(const PolicyCommandSnapshot& commandSnapshot) {
  legged_msgs::msg::JointControlData msg;
  msg.joint_position.resize(kJointCount);
  msg.joint_velocity.assign(kJointCount, 0.0);
  msg.joint_torque.assign(kJointCount, 0.0);
  msg.kp = settings_.kpRatio;
  msg.kd = settings_.kdRatio;

  for (std::size_t i = 0; i < kJointCount; ++i) {
    msg.joint_position[i] = commandSnapshot.jointTarget[i];
  }

  for (std::size_t actionIndex = 0; actionIndex < commandSnapshot.actionClipped.size(); ++actionIndex) {
    lastAction_[actionIndex] = commandSnapshot.actionClipped[actionIndex];
  }

  jointControlPublisher_->publish(msg);
  ++debugDumpedPolicySteps_;
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
  auto command = activeVelocityCommand();
  if (settings_.invertVelocityCommandXY) {
    command.linear.x = -command.linear.x;
    command.linear.y = -command.linear.y;
  }
  if (settings_.velocityCommandCap > 0.0) {
    command.linear.x = clampMagnitude(command.linear.x, settings_.velocityCommandCap);
    command.linear.y = clampMagnitude(command.linear.y, settings_.velocityCommandCap);
    command.angular.z = clampMagnitude(command.angular.z, settings_.velocityCommandCap);
  }
  return command;
}
