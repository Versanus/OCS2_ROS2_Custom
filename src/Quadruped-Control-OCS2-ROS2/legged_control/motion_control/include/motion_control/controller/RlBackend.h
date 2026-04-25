#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "legged_msgs/msg/joint_control_data.hpp"
#include "legged_msgs/msg/simulator_sensor_data.hpp"
#include "legged_msgs/msg/simulator_state_data.hpp"
#include "motion_control/common/Types.h"
#include "motion_control/controller/ControllerBackend.h"
#include "motion_control/controller/PolicyRunner.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class RlBackend final : public ControllerBackend {
 public:
  explicit RlBackend(const rclcpp::Node::SharedPtr& node);
  ~RlBackend() override;

  bool configure(const ControllerConfig& config) override;
  void launch() override;
  const char* name() const override;

 private:
  enum class ControlCommand {
    ActivatePolicyLegacy = 0,
    Hold = 1,
    RecoveryPose = 2,
    SitDown = 3,
    ZeroTorque = 4,
    Stand = 6,
    ActivatePolicy = 7,
  };

  enum class ControlState {
    Hold = 1,
    RecoveryPose = 2,
    SitDown = 3,
    ZeroTorque = 5,
    Stand = 6,
    Policy = 7,
  };

  enum class ObservationLayout {
    Legacy,
    Policy,
  };

  enum class JointPositionObservation {
    RelativeToDefault,
    Raw,
  };

  enum class FeedbackJointStateTransform {
    None,
    QuadMiniHardware,
  };

  struct Settings {
    std::string modelPath;
    std::string inputName{"obs"};
    std::string outputName{"continuous_actions"};
    double policyHz = 50.0;
    double actionScale = 0.50;
    double actionClip = 1.0;
    double actionRateLimit = 0.0;
    double kpRatio = 1.0;
    double kdRatio = 1.0;
    double poseKpRatio = 4.0;
    double poseKdRatio = 2.0;
    double stateTimeoutSec = 0.25;
    double commandTimeoutSec = 1.0;
    double startupHoldSec = 1.0;
    double standTransitionSec = 1.5;
    double sitTransitionSec = 1.5;
    double commandActivationThreshold = 0.02;
    double velocityCommandCap = 0.0;
    bool requireCommandForPolicy = false;
    bool holdStandWhenPolicyIdle = false;
    std::size_t observationDim = 48;
    std::size_t actionDim = 12;
    ObservationLayout observationLayout = ObservationLayout::Legacy;
    JointPositionObservation jointPositionObservation = JointPositionObservation::RelativeToDefault;
    FeedbackJointStateTransform feedbackJointStateTransform = FeedbackJointStateTransform::None;
    bool debugDumpEnabled = false;
    std::string debugDumpDir;
    std::size_t debugDumpMaxSteps = 0;
    std::vector<std::size_t> actionToJointIndex;
    std::vector<std::size_t> observationToJointIndex;
  };

  struct ObservationSnapshot {
    double sensorTime = 0.0;
    std::vector<float> imuQuaternion;
    std::vector<float> baseLinearVelocity;
    std::vector<float> baseAngularVelocity;
    std::vector<float> projectedGravity;
    std::vector<float> jointPositionRelativeToDefault;
    std::vector<float> jointVelocity;
    std::vector<float> previousAction;
    std::vector<float> command;
    std::vector<float> observation;
  };

  struct PolicyCommandSnapshot {
    std::vector<float> actionRaw;
    std::vector<float> actionClipped;
    std::vector<float> jointTarget;
  };

  bool loadSettings(const ControllerConfig& config);
  bool loadPoses(const std::string& rlConfigFile);
  static ObservationLayout parseObservationLayout(const std::string& value);
  static JointPositionObservation parseJointPositionObservation(const std::string& value);
  static FeedbackJointStateTransform parseFeedbackJointStateTransform(const std::string& value);
  void setupRosInterfaces();
  void initializeStandState();

  void stateCallback(const legged_msgs::msg::SimulatorStateData::SharedPtr msg);
  void sensorCallback(const legged_msgs::msg::SimulatorSensorData::SharedPtr msg);
  void velocityCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void emergencyOverrideCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void policyTimerCallback();

  bool hasFreshState() const;
  bool hasFreshSensor() const;
  bool hasActiveVelocityCommand() const;
  bool latchHoldPoseFromLatestState();
  bool fillPoseFromLatestState(ocs2::vector_t& pose) const;
  void resetPoseTransition();
  void beginPoseTransition(const ocs2::vector_t& targetPose, double durationSec);
  ocs2::vector_t transitionPoseCommand() const;
  std::string poseSummary(const ocs2::vector_t& pose) const;
  bool buildObservationSnapshot(ObservationSnapshot& snapshot) const;
  void transformFeedbackJointState(std::vector<double>& jointPositions, std::vector<double>& jointVelocities) const;
  PolicyCommandSnapshot buildPolicyCommandSnapshot(const std::vector<float>& action) const;
  void prepareDebugDumpDirectory();
  void dumpPolicyStep(const ObservationSnapshot& observationSnapshot,
                      const PolicyCommandSnapshot& commandSnapshot) const;
  void publishPolicyCommand(const std::vector<float>& action);
  void publishPolicyCommand(const PolicyCommandSnapshot& commandSnapshot);
  void publishPoseCommand(const ocs2::vector_t& pose, double kpRatio, double kdRatio);
  void publishZeroTorqueCommand();
  void publishEmergencyOverrideState() const;
  ocs2::vector_t poseForControlState() const;
  geometry_msgs::msg::Twist activeVelocityCommand() const;
  geometry_msgs::msg::Twist policyVelocityCommand() const;

  rclcpp::Node::SharedPtr node_;
  ControllerConfig controllerConfig_;
  Settings settings_;
  std::unique_ptr<PolicyRunner> policyRunner_;

  rclcpp::Publisher<legged_msgs::msg::JointControlData>::SharedPtr jointControlPublisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr emergencyOverrideStatePublisher_;
  rclcpp::Subscription<legged_msgs::msg::SimulatorStateData>::SharedPtr simulatorStateSubscriber_;
  rclcpp::Subscription<legged_msgs::msg::SimulatorSensorData>::SharedPtr simulatorSensorSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocityCommandSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr emergencyOverrideSubscriber_;
  rclcpp::TimerBase::SharedPtr policyTimer_;

  legged_msgs::msg::SimulatorStateData latestState_;
  legged_msgs::msg::SimulatorSensorData latestSensor_;
  geometry_msgs::msg::Twist latestVelocityCommand_;
  rclcpp::Time lastStateReceiptTime_;
  rclcpp::Time lastSensorReceiptTime_;
  rclcpp::Time lastCommandReceiptTime_;
  rclcpp::Time firstFreshStateTime_;
  bool hasState_ = false;
  bool hasSensor_ = false;
  bool startupHoldStarted_ = false;
  bool hasHoldPose_ = false;

  ControlState controlState_ = ControlState::Stand;
  rclcpp::Time poseTransitionStartTime_;
  double poseTransitionDurationSec_ = 0.0;
  ocs2::vector_t defaultJointState_ = ocs2::vector_t::Zero(12);
  ocs2::vector_t holdJointState_ = ocs2::vector_t::Zero(12);
  ocs2::vector_t poseTransitionStartPose_ = ocs2::vector_t::Zero(12);
  ocs2::vector_t poseTransitionTargetPose_ = ocs2::vector_t::Zero(12);
  ocs2::vector_t standJointState_ = ocs2::vector_t::Zero(12);
  ocs2::vector_t recoveryJointState_ = ocs2::vector_t::Zero(12);
  ocs2::vector_t sitJointState_ = ocs2::vector_t::Zero(12);
  std::vector<float> lastAction_;
  std::filesystem::path debugDumpDir_;
  std::size_t debugDumpedPolicySteps_ = 0;
};
