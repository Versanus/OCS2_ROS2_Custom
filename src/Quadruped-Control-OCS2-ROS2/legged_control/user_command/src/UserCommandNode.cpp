#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <deque>
#include <thread>
#include <stdexcept>
#include <unordered_map>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "std_msgs/msg/int32.hpp"
#include "user_command/goal/TargetTrajectoriesKeyboardPublisher.h"
#include "user_command/gait/GaitKeyboardPublisher.h"
#include "rclcpp/rclcpp.hpp"


// Split command line
std::vector<std::string> splitByColon(const std::string& input) {
    std::vector<std::string> result;
    std::string token;
    std::istringstream tokenStream(input);


    while (std::getline(tokenStream, token, ':')) {
        result.push_back(token);
    }
    return result;
}

// erase the space in the front of command line
std::string trimLeft(const std::string& str) {
    std::string result = str;
    result.erase(result.begin(), std::find_if(result.begin(), result.end(), [](unsigned char ch) {
        return !std::isspace(ch);  
    }));
    return result;
}

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

ocs2::vector_t parseNumericCommand(const std::string& input, size_t expectedSize) {
  const std::vector<std::string> words = ocs2::stringToWords(input);
  ocs2::vector_t command = ocs2::vector_t::Zero(expectedSize);
  for (size_t i = 0; i < std::min(words.size(), expectedSize); ++i) {
    command(i) = static_cast<ocs2::scalar_t>(std::stod(words[i]));
  }
  return command;
}

ocs2::vector_t applyAxisScales(const ocs2::vector_t& command, const ocs2::vector_t& axisScales) {
  if (command.size() != axisScales.size()) {
    throw std::invalid_argument("Command axis scale size mismatch.");
  }
  return command.cwiseProduct(axisScales);
}

void applyAxisScalesToGoalCommand(ocs2::vector_t& goalCommand, const ocs2::vector_t& axisScales) {
  if (goalCommand.size() < 4 || axisScales.size() < 3) {
    throw std::invalid_argument("Goal command axis scale size mismatch.");
  }
  goalCommand(0) *= axisScales(0);
  goalCommand(1) *= axisScales(1);
  goalCommand(3) *= axisScales(2);
}

enum class UserCommandMode {
  Goal,
  Velocity
};

enum class ControllerType {
  Mpc,
  Rl
};

std::string modeToString(UserCommandMode mode) {
  return mode == UserCommandMode::Goal ? "goal" : "vel";
}

ControllerType controllerTypeFromString(std::string value) {
  value = toLower(trimLeft(value));
  if (value == "rl") {
    return ControllerType::Rl;
  }
  return ControllerType::Mpc;
}

struct TeleopCommandState {
  ocs2::scalar_t forwardAxis = 0.0;
  ocs2::scalar_t lateralAxis = 0.0;
  ocs2::scalar_t yawAxis = 0.0;
  bool holdPositionActive = true;
  bool stabilizeModeActive = false;
  bool estopActive = false;

  void resetAxes() {
    forwardAxis = 0.0;
    lateralAxis = 0.0;
    yawAxis = 0.0;
  }
};

enum class MotionInputSource {
  Keyboard,
  Gamepad
};

struct GamepadConfig {
  bool enabled = true;
  std::string device = "/dev/input/js0";
  double axisDeadzone = 0.20;
  double digitalPressThreshold = 0.60;
  int axisLateral = 0;
  int axisForward = 1;
  int axisYaw = 3;
  int dpadHorizontalAxis = 6;
  int dpadVerticalAxis = 7;
  int leftTriggerAxis = 2;
  int rightTriggerAxis = 5;
  int buttonB = 1;
  int buttonX = 2;
  int buttonY = 3;
  int buttonLb = 4;
  int buttonRb = 5;
};

class GamepadTeleopInput {
 public:
  GamepadTeleopInput(const GamepadConfig& config, const rclcpp::Logger& logger)
      : config_(config), logger_(logger) {
    if (config_.enabled) {
      tryOpenDevice();
    }
  }

  ~GamepadTeleopInput() {
    closeDevice();
  }

  void poll() {
    if (!config_.enabled) {
      return;
    }

    ensureDeviceOpen();
    if (fd_ < 0) {
      return;
    }

    while (true) {
      js_event event{};
      const ssize_t bytesRead = ::read(fd_, &event, sizeof(event));
      if (bytesRead == static_cast<ssize_t>(sizeof(event))) {
        const unsigned char type = event.type & ~JS_EVENT_INIT;
        if (type == JS_EVENT_AXIS) {
          handleAxisEvent(event.number, event.value);
        } else if (type == JS_EVENT_BUTTON) {
          handleButtonEvent(event.number, event.value);
        }
        continue;
      }

      if (bytesRead < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      }

      handleDisconnect();
      break;
    }
  }

  char popNextMappedKey() {
    if (pendingKeys_.empty()) {
      return '\0';
    }

    const char key = pendingKeys_.front();
    pendingKeys_.pop_front();
    return key;
  }

  ocs2::vector_t currentMotionAxes() const {
    ocs2::vector_t axes = ocs2::vector_t::Zero(3);
    if (fd_ < 0) {
      return axes;
    }

    axes(0) = -applyDeadzone(normalizedAxisValue(config_.axisForward), config_.axisDeadzone);
    axes(1) = -applyDeadzone(normalizedAxisValue(config_.axisLateral), config_.axisDeadzone);
    axes(2) = -applyDeadzone(normalizedAxisValue(config_.axisYaw), config_.axisDeadzone);
    return axes;
  }

  bool hasActiveMotionInput() const {
    return !currentMotionAxes().isZero(1e-3);
  }

  void setStabilizeToggleState(bool active) {
    stabilizeToggleState_ = active;
  }

 private:
  static double normalizeAxis(int16_t value) {
    const double denominator = value >= 0 ? 32767.0 : 32768.0;
    return std::clamp(static_cast<double>(value) / denominator, -1.0, 1.0);
  }

  static double applyDeadzone(double value, double deadzone) {
    const double magnitude = std::abs(value);
    if (magnitude <= deadzone) {
      return 0.0;
    }

    const double scaledMagnitude = (magnitude - deadzone) / std::max(1e-6, 1.0 - deadzone);
    return std::copysign(std::clamp(scaledMagnitude, 0.0, 1.0), value);
  }

  void ensureDeviceOpen() {
    if (fd_ >= 0) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < nextOpenAttemptTime_) {
      return;
    }

    tryOpenDevice();
  }

  void tryOpenDevice() {
    fd_ = ::open(config_.device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
      if (!missingDeviceReported_) {
        RCLCPP_INFO(logger_, "Optional gamepad '%s' not available. Keyboard teleop stays active.", config_.device.c_str());
        missingDeviceReported_ = true;
      }
      nextOpenAttemptTime_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      return;
    }

    missingDeviceReported_ = false;
    nextOpenAttemptTime_ = std::chrono::steady_clock::time_point{};
    RCLCPP_INFO(logger_, "Gamepad teleop connected on %s.", config_.device.c_str());
  }

  void closeDevice() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void handleDisconnect() {
    if (fd_ >= 0) {
      RCLCPP_WARN(logger_, "Gamepad teleop disconnected from %s. Falling back to keyboard.", config_.device.c_str());
    }
    closeDevice();
    axisStates_.clear();
    pendingKeys_.clear();
    resetDigitalStates();
    nextOpenAttemptTime_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  }

  double normalizedAxisValue(int axisId) const {
    const auto it = axisStates_.find(axisId);
    if (it == axisStates_.end()) {
      return 0.0;
    }
    return normalizeAxis(it->second);
  }

  void handleAxisEvent(unsigned int axisNumber, int16_t value) {
    axisStates_[static_cast<int>(axisNumber)] = value;
    const double normalizedValue = normalizeAxis(value);

    if (static_cast<int>(axisNumber) == config_.dpadHorizontalAxis) {
      queueKeyOnRisingEdge(normalizedValue <= -config_.digitalPressThreshold, dpadLeftPressed_, '1');
      queueKeyOnRisingEdge(normalizedValue >= config_.digitalPressThreshold, dpadRightPressed_, '2');
      return;
    }

    if (static_cast<int>(axisNumber) == config_.dpadVerticalAxis) {
      queueKeyOnRisingEdge(normalizedValue <= -config_.digitalPressThreshold, dpadUpPressed_, 'o');
      queueKeyOnRisingEdge(normalizedValue >= config_.digitalPressThreshold, dpadDownPressed_, 'l');
      return;
    }

    if (static_cast<int>(axisNumber) == config_.leftTriggerAxis) {
      queueKeyOnRisingEdge(normalizedValue >= config_.digitalPressThreshold, leftTriggerPressed_, '-');
      return;
    }

    if (static_cast<int>(axisNumber) == config_.rightTriggerAxis) {
      queueKeyOnRisingEdge(normalizedValue >= config_.digitalPressThreshold, rightTriggerPressed_, '+');
    }
  }

  void handleButtonEvent(unsigned int buttonNumber, int16_t value) {
    const bool pressed = value != 0;
    const int button = static_cast<int>(buttonNumber);
    if (button == config_.buttonB) {
      queueKeyOnRisingEdge(pressed, buttonBPressed_, 'z');
    } else if (button == config_.buttonLb) {
      queueKeyOnRisingEdge(pressed, buttonLbPressed_, ' ');
    } else if (button == config_.buttonRb) {
      queueKeyOnRisingEdge(pressed, buttonRbPressed_, '0');
    } else if (button == config_.buttonX) {
      queueKeyOnRisingEdge(pressed, buttonXPressed_, 'c');
    } else if (button == config_.buttonY) {
      queueToggleOnRisingEdge(pressed, buttonYPressed_, 'y', 't');
    }
  }

  void queueKeyOnRisingEdge(bool currentlyPressed, bool& previousPressed, char mappedKey) {
    if (currentlyPressed && !previousPressed) {
      pendingKeys_.push_back(mappedKey);
    }
    previousPressed = currentlyPressed;
  }

  void queueToggleOnRisingEdge(bool currentlyPressed, bool& previousPressed, char onKey, char offKey) {
    if (currentlyPressed && !previousPressed) {
      pendingKeys_.push_back(stabilizeToggleState_ ? offKey : onKey);
      stabilizeToggleState_ = !stabilizeToggleState_;
    }
    previousPressed = currentlyPressed;
  }

  void resetDigitalStates() {
    dpadLeftPressed_ = false;
    dpadRightPressed_ = false;
    dpadUpPressed_ = false;
    dpadDownPressed_ = false;
    leftTriggerPressed_ = false;
    rightTriggerPressed_ = false;
    buttonBPressed_ = false;
    buttonXPressed_ = false;
    buttonYPressed_ = false;
    buttonLbPressed_ = false;
    buttonRbPressed_ = false;
  }

  GamepadConfig config_;
  rclcpp::Logger logger_;
  int fd_ = -1;
  bool missingDeviceReported_ = false;
  bool stabilizeToggleState_ = false;
  bool dpadLeftPressed_ = false;
  bool dpadRightPressed_ = false;
  bool dpadUpPressed_ = false;
  bool dpadDownPressed_ = false;
  bool leftTriggerPressed_ = false;
  bool rightTriggerPressed_ = false;
  bool buttonBPressed_ = false;
  bool buttonXPressed_ = false;
  bool buttonYPressed_ = false;
  bool buttonLbPressed_ = false;
  bool buttonRbPressed_ = false;
  std::chrono::steady_clock::time_point nextOpenAttemptTime_{};
  std::unordered_map<int, int16_t> axisStates_;
  std::deque<char> pendingKeys_;
};

class ScopedRawTerminalMode {
 public:
  explicit ScopedRawTerminalMode(int fd = STDIN_FILENO) : fd_(fd) {
    if (tcgetattr(fd_, &originalState_) != 0) {
      throw std::runtime_error("Failed to read terminal settings.");
    }

    termios rawState = originalState_;
    rawState.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    rawState.c_cc[VMIN] = 0;
    rawState.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &rawState) != 0) {
      throw std::runtime_error("Failed to enable raw terminal mode.");
    }

    tcflush(fd_, TCIFLUSH);
    active_ = true;
  }

  ~ScopedRawTerminalMode() {
    if (active_) {
      tcsetattr(fd_, TCSANOW, &originalState_);
    }
  }

  ScopedRawTerminalMode(const ScopedRawTerminalMode&) = delete;
  ScopedRawTerminalMode& operator=(const ScopedRawTerminalMode&) = delete;

 private:
  int fd_;
  bool active_ = false;
  termios originalState_{};
};

char readSingleKey(std::chrono::milliseconds timeout) {
  pollfd pfd{};
  pfd.fd = STDIN_FILENO;
  pfd.events = POLLIN;

  const int pollResult = poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (pollResult <= 0 || (pfd.revents & POLLIN) == 0) {
    return '\0';
  }

  char key = '\0';
  const ssize_t bytesRead = ::read(STDIN_FILENO, &key, 1);
  return bytesRead == 1 ? key : '\0';
}

ocs2::vector_t normalizedAxisFromKey(char key) {
  ocs2::vector_t axis = ocs2::vector_t::Zero(3);
  switch (key) {
    case 'w':
      axis(0) = 1.0;
      break;
    case 's':
      axis(0) = -1.0;
      break;
    case 'a':
      axis(1) = 1.0;
      break;
    case 'd':
      axis(1) = -1.0;
      break;
    case 'q':
      axis(2) = 1.0;
      break;
    case 'e':
      axis(2) = -1.0;
      break;
    default:
      break;
  }
  return axis;
}

bool isMotionKey(char key) {
  switch (key) {
    case 'w':
    case 'a':
    case 's':
    case 'd':
    case 'q':
    case 'e':
      return true;
    default:
      return false;
  }
}

std::string gaitCommandFromKey(char key) {
  switch (key) {
    case '1':
      return "stance";
    case '2':
      return "standing_trot";
    case '3':
      return "flying_trot";
    case '4':
      return "static_walking";
    case '5':
      return "pawup";
    case '6':
      return "fast_flying_trot";
    default:
      return "";
  }
}

ocs2::vector_t clampGoalCommandForStance(const ocs2::vector_t& goalCommand, bool& wasClamped) {
  const ocs2::vector_t stanceLimits =
      (ocs2::vector_t(4) << 2.0, 1.0, 0.04, 2.0).finished();
  const ocs2::vector_t clampedGoal =
      goalCommand.cwiseMin(stanceLimits).cwiseMax(-stanceLimits);
  wasClamped = !clampedGoal.isApprox(goalCommand);
  return clampedGoal;
}

ocs2::vector_t clampVelocityCommandForStance(const ocs2::vector_t& velocityCommand, bool& wasClamped) {
  const ocs2::vector_t stanceVelocityLimits =
      (ocs2::vector_t(3) << 2.0, 1.0, 2.0).finished();
  const ocs2::vector_t clampedVelocity =
      velocityCommand.cwiseMin(stanceVelocityLimits).cwiseMax(-stanceVelocityLimits);
  wasClamped = !clampedVelocity.isApprox(velocityCommand);
  return clampedVelocity;
}

enum class ControlCommand {
  ActivateMpc = 0,
  Hold = 1,
  RecoveryPose = 2,
  SitDown = 3,
  ZeroTorque = 4,
  ActivateRlStand = 6,
  ActivateRlPolicy = 7,
};

void publishEmergencyOverrideCommand(
    const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr& publisher,
    ControlCommand command) {
  auto msg = std_msgs::msg::Int32();
  msg.data = static_cast<int>(command);
  publisher->publish(msg);
}

enum class ControlState {
  Mpc = 0,
  Hold = 1,
  RecoveryPose = 2,
  SitDown = 3,
  Sitting = 4,
  ZeroTorque = 5,
  RlStand = 6,
  RlPolicy = 7,
};

bool isWalkingControlActive(ControlState state, ControllerType controllerType) {
  if (controllerType == ControllerType::Rl) {
    return state == ControlState::RlPolicy;
  }
  return state == ControlState::Mpc;
}

void printVelocityModeHelp(ControllerType controllerType, const std::string& status = "") {
  std::cout << "\033[2J\033[H"
      << "\nVelocity mode keyboard control:\n"
      << "  w/s : forward/backward\n"
      << "  a/d : left/right strafe\n"
      << "  q/e : yaw left/right\n";
  if (controllerType == ControllerType::Rl) {
    std::cout
        << "  startup: RL home position hold from policyDefaultJointState\n"
        << "  1 : return to RL home position hold\n"
        << "  2 : activate RL policy from home hold\n"
        << "  0 : raw recovery pose\n"
        << "  z : sit down with strong PD\n"
        << "  c : true zero torque mode\n"
        << "  y : stabilize in place using current x/y reference\n"
        << "  t : resume walking mode inside vel mode\n"
        << "  o/l : raise/lower desired height slowly\n"
        << "  +/- : increase/decrease speeds\n"
        << "  space : hold the current joints from the current pose\n"
        << "  g : exit velocity mode back to goal mode\n"
        << "\nOptional 8Bitdo gamepad control:\n"
        << "  left stick : w/a/s/d analog motion after RL is active\n"
        << "  right stick X : q/e analog yaw after RL is active\n"
        << "  d-pad left/right : 1/2\n"
        << "  d-pad up/down : o/l\n"
        << "  B : z, LB : space(hold), RB : 0, X : c\n"
        << "  Y : stabilize toggle (y/t), LT/RT : -/+\n"
        << "Press 2 to arm RL walking from the startup home hold.\n"
        << "Motion keys only move the robot once RL policy is active.\n";
  } else {
    std::cout
        << "  1 : stance\n"
        << "  2 : standing_trot\n"
        << "  3 : flying_trot\n"
        << "  4 : static_walking\n"
        << "  5 : pawup\n"
        << "  6 : fast_flying_trot\n"
        << "  startup: system begins in E-stop, press 1 to enable MPC\n"
        << "  0 : raw recovery pose (from safe non-MPC states)\n"
        << "      not allowed while sit-down is still in progress\n"
        << "  z : sit down with strong PD (only from MPC stance)\n"
        << "  c : true zero torque mode (from hold / recovery / sitting)\n"
        << "  y : stabilize in place using current x/y reference\n"
        << "  t : resume walking mode inside vel mode\n"
        << "  o/l : raise/lower desired height slowly\n"
        << "  +/- : increase/decrease speeds\n"
        << "  r : switch to stance with safe clamped motion\n"
        << "  space : hold current joints and refresh the background target to the current pose\n"
        << "  g : exit velocity mode back to goal mode\n"
        << "\nOptional 8Bitdo gamepad control:\n"
        << "  left stick : w/a/s/d analog motion\n"
        << "  right stick X : q/e analog yaw\n"
        << "  d-pad left/right : 1/2\n"
        << "  d-pad up/down : o/l\n"
        << "  B : z, LB : space(hold), RB : 0, X : c\n"
        << "  Y : stabilize toggle (y/t), LT/RT : -/+\n"
        << "Pressing 1 from HOLD returns to stance from the current pose.\n"
        << "The last motion key stays latched until you overwrite it or clear it with r.\n"
        << "The last active motion source wins, so you can switch between keyboard and controller.\n";
  }
  if (!status.empty()) {
    std::cout << "\nStatus: " << status << "\n";
  }
  std::cout << std::endl;
}

void updateTeleopAxisFromKeyboard(TeleopCommandState& teleopState, char key) {
  const ocs2::vector_t axis = normalizedAxisFromKey(key);
  teleopState.forwardAxis = axis(0);
  teleopState.lateralAxis = axis(1);
  teleopState.yawAxis = axis(2);
}

ocs2::vector_t composeVelocityCommand(const TeleopCommandState& teleopState, ocs2::scalar_t linearSpeed,
                                      ocs2::scalar_t lateralSpeed, ocs2::scalar_t yawSpeed,
                                      const ocs2::vector_t& commandAxisScales) {
  ocs2::vector_t velocityCommand = ocs2::vector_t::Zero(3);
  velocityCommand(0) = teleopState.forwardAxis * linearSpeed;
  velocityCommand(1) = teleopState.lateralAxis * lateralSpeed;
  velocityCommand(2) = teleopState.yawAxis * yawSpeed;
  return applyAxisScales(velocityCommand, commandAxisScales);
}

ocs2::vector_t stepTowardVelocityCommand(const ocs2::vector_t& currentVelocityCommand,
                                         const ocs2::vector_t& targetVelocityCommand,
                                         const ocs2::vector_t& accelerationLimits,
                                         ocs2::scalar_t dt) {
  ocs2::vector_t nextVelocityCommand = currentVelocityCommand;
  for (int i = 0; i < currentVelocityCommand.size(); ++i) {
    const ocs2::scalar_t maxStep = accelerationLimits(i) * dt;
    const ocs2::scalar_t velocityError = targetVelocityCommand(i) - currentVelocityCommand(i);
    nextVelocityCommand(i) = currentVelocityCommand(i) + std::clamp(velocityError, -maxStep, maxStep);
  }
  return nextVelocityCommand;
}

void runVelocityKeyboardMode(TargetTrajectoriesKeyboardPublisher& targetPoseCommand,
                             GaitKeyboardPublisher& gaitCommand,
                             UserCommandMode& currentMode,
                             std::string& activeGaitCommand,
                             ocs2::scalar_t initialLinearSpeed,
                             ocs2::scalar_t initialLateralSpeed,
                             ocs2::scalar_t initialYawSpeed,
                             const ocs2::vector_t& accelerationLimits,
                             const ocs2::vector_t& commandAxisScales,
                             ControllerType controllerType,
                             const GamepadConfig& gamepadConfig,
                             const rclcpp::Node::SharedPtr& node,
                             const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr& emergencyOverridePublisher,
                             ControlState& controlState) {
  auto showVelocityModeHelp = [&](const std::string& status = "") {
    printVelocityModeHelp(controllerType, status);
  };

  showVelocityModeHelp("Velocity mode active.");

  ScopedRawTerminalMode rawTerminalMode;
  GamepadTeleopInput gamepadInput(gamepadConfig, node->get_logger());
  MotionInputSource activeMotionSource = MotionInputSource::Keyboard;
  bool gamepadMotionRequiresRecenter = false;

  ocs2::scalar_t linearSpeed = initialLinearSpeed;
  ocs2::scalar_t lateralSpeed = initialLateralSpeed;
  ocs2::scalar_t yawSpeed = initialYawSpeed;
  const ocs2::scalar_t linearStep = 0.05;
  const ocs2::scalar_t yawStep = 0.1;
  const ocs2::scalar_t heightStep = 0.005;
  const ocs2::scalar_t minLinearSpeed = 0.05;
  const ocs2::scalar_t minYawSpeed = 0.1;
  ocs2::vector_t targetVelocityCommand = ocs2::vector_t::Zero(3);
  ocs2::vector_t filteredVelocityCommand = ocs2::vector_t::Zero(3);
  auto lastVelocityUpdateTime = std::chrono::steady_clock::now();
  TeleopCommandState teleopState;
  teleopState.resetAxes();
  teleopState.estopActive = !isWalkingControlActive(controlState, controllerType);

  while (rclcpp::ok() && currentMode == UserCommandMode::Velocity) {
    rclcpp::spin_some(node);
    teleopState.estopActive = !isWalkingControlActive(controlState, controllerType);
    gamepadInput.poll();

    const char rawKey = readSingleKey(std::chrono::milliseconds(50));
    const char keyboardKey = rawKey == '\0' ? '\0' : static_cast<char>(std::tolower(static_cast<unsigned char>(rawKey)));
    const char key = keyboardKey != '\0' ? keyboardKey : gamepadInput.popNextMappedKey();
    bool keyboardMotionKeyProcessed = false;
    if (key != '\0') {
      if (teleopState.estopActive) {
        if (controllerType == ControllerType::Rl) {
          if (key == ' ') {
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::Hold);
            controlState = ControlState::Hold;
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            targetPoseCommand.publishHoldPositionCommand(false);
            showVelocityModeHelp("RL hold active from the current pose.");
          } else if (key == '0') {
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::RecoveryPose);
            controlState = ControlState::RecoveryPose;
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            targetPoseCommand.publishHoldPositionCommand(false);
            showVelocityModeHelp("RL recovery pose requested.");
          } else if (key == '1') {
            activeGaitCommand = "stance";
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            targetPoseCommand.publishHoldPositionCommand(false);
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::ActivateRlStand);
            controlState = ControlState::RlStand;
            showVelocityModeHelp("RL home position hold requested.");
          } else if (key == '2') {
            if (controlState == ControlState::Hold || controlState == ControlState::RlStand) {
              targetVelocityCommand.setZero();
              filteredVelocityCommand.setZero();
              teleopState.resetAxes();
              teleopState.holdPositionActive = true;
              teleopState.stabilizeModeActive = false;
              gamepadMotionRequiresRecenter = true;
              publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::ActivateRlPolicy);
              controlState = ControlState::RlPolicy;
              showVelocityModeHelp("RL policy requested. Use motion keys to move.");
            } else if (controlState == ControlState::RlPolicy) {
              showVelocityModeHelp("RL policy is already active.");
            } else {
              showVelocityModeHelp("Return to RL home hold with 1 before activating policy.");
            }
          } else if (key == 'z') {
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::SitDown);
            controlState = ControlState::SitDown;
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            targetPoseCommand.publishHoldPositionCommand(false);
            showVelocityModeHelp("RL sit-down requested.");
          } else if (key == 'c') {
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::ZeroTorque);
            controlState = ControlState::ZeroTorque;
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            targetPoseCommand.publishHoldPositionCommand(false);
            showVelocityModeHelp("RL zero torque requested.");
          } else {
            showVelocityModeHelp("RL safe mode active. Use 1=home hold, 2=RL, 0=recovery, z=sit, c=zero torque, space=hold.");
          }
          continue;
        }

        if (key == ' ') {
          publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::Hold);
          showVelocityModeHelp("Hold active: actuator output is locked, MPC/reference keeps updating in the background.");
        } else if (key == '0') {
          if (controlState == ControlState::SitDown) {
            showVelocityModeHelp("Sit-down is still in progress. Wait for sitting, or press space to hold first.");
          } else {
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::RecoveryPose);
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            showVelocityModeHelp("Recovery pose active.");
          }
        } else if (key == '1') {
          if (controlState == ControlState::Hold) {
            std::string stanceCommand = "stance";
            gaitCommand.publishKeyboardCommand(stanceCommand);
            activeGaitCommand = "stance";
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            targetPoseCommand.publishHoldPositionCommand(false);
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::ActivateMpc);
            showVelocityModeHelp("MPC stance resume requested from the current pose.");
          } else {
            std::string stanceCommand = "stance";
            gaitCommand.publishKeyboardCommand(stanceCommand);
            activeGaitCommand = "stance";
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            targetPoseCommand.publishHoldPositionCommand(false);
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::ActivateMpc);
            showVelocityModeHelp("MPC stance activation requested.");
          }
        } else if (key == 'c') {
          if (controlState == ControlState::Sitting || controlState == ControlState::Hold ||
              controlState == ControlState::RecoveryPose || controlState == ControlState::ZeroTorque) {
            publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::ZeroTorque);
            targetVelocityCommand.setZero();
            filteredVelocityCommand.setZero();
            teleopState.resetAxes();
            teleopState.holdPositionActive = true;
            teleopState.stabilizeModeActive = false;
            gamepadMotionRequiresRecenter = true;
            showVelocityModeHelp("True zero torque mode active.");
          } else {
            showVelocityModeHelp("Zero torque is only allowed from a safe non-MPC posture.");
          }
        } else if (key == 'z') {
          if (controlState == ControlState::Sitting || controlState == ControlState::SitDown) {
            showVelocityModeHelp("Robot is already in the sit-down path.");
          } else {
            showVelocityModeHelp("Sit-down is only available while MPC stance is active.");
          }
        } else if (key == 'r') {
          showVelocityModeHelp("Soft stance command is only available while MPC is active.");
        } else {
          showVelocityModeHelp("Safe mode active. Use space=hold, 0=recovery, 1=resume MPC, c=zero torque.");
        }
        continue;
      }

      if (key == ' ') {
        publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::Hold);
        if (controllerType == ControllerType::Rl) {
          controlState = ControlState::Hold;
        }
        teleopState.estopActive = true;
        targetVelocityCommand.setZero();
        filteredVelocityCommand.setZero();
        teleopState.resetAxes();
        teleopState.holdPositionActive = true;
        teleopState.stabilizeModeActive = false;
        gamepadMotionRequiresRecenter = true;
        targetPoseCommand.publishHoldPositionCommand(false);
        showVelocityModeHelp("Hold active: actuator output is locked and the background target is refreshed to the current pose.");
        continue;
      }

      if (controllerType == ControllerType::Rl) {
        if (key == '0') {
          publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::RecoveryPose);
          controlState = ControlState::RecoveryPose;
          targetVelocityCommand.setZero();
          filteredVelocityCommand.setZero();
          teleopState.resetAxes();
          teleopState.holdPositionActive = true;
          teleopState.stabilizeModeActive = false;
          gamepadMotionRequiresRecenter = true;
          targetPoseCommand.publishHoldPositionCommand(false);
          showVelocityModeHelp("RL recovery pose requested.");
          continue;
        }
        if (key == '1') {
          activeGaitCommand = "stance";
          publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::ActivateRlStand);
          controlState = ControlState::RlStand;
          targetVelocityCommand.setZero();
          filteredVelocityCommand.setZero();
          teleopState.resetAxes();
          teleopState.holdPositionActive = true;
          teleopState.stabilizeModeActive = false;
          gamepadMotionRequiresRecenter = true;
          targetPoseCommand.publishHoldPositionCommand(false);
          showVelocityModeHelp("Switched back to RL home position hold.");
          continue;
        }
        if (key == '2') {
          showVelocityModeHelp("RL policy is already active. Use motion keys to move.");
          continue;
        }
        if (key == 'z') {
          publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::SitDown);
          controlState = ControlState::SitDown;
          targetVelocityCommand.setZero();
          filteredVelocityCommand.setZero();
          teleopState.resetAxes();
          teleopState.holdPositionActive = true;
          teleopState.stabilizeModeActive = false;
          gamepadMotionRequiresRecenter = true;
          targetPoseCommand.publishHoldPositionCommand(false);
          showVelocityModeHelp("RL sit-down requested.");
          continue;
        }
        if (key == 'c') {
          publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::ZeroTorque);
          controlState = ControlState::ZeroTorque;
          targetVelocityCommand.setZero();
          filteredVelocityCommand.setZero();
          teleopState.resetAxes();
          teleopState.holdPositionActive = true;
          teleopState.stabilizeModeActive = false;
          gamepadMotionRequiresRecenter = true;
          targetPoseCommand.publishHoldPositionCommand(false);
          showVelocityModeHelp("RL zero torque requested.");
          continue;
        }
      }

      const std::string gaitCommandString =
          controllerType == ControllerType::Rl ? std::string{} : gaitCommandFromKey(key);
      if (!gaitCommandString.empty()) {
        std::string gaitCommandValue = gaitCommandString;
        gaitCommand.publishKeyboardCommand(gaitCommandValue);
        activeGaitCommand = gaitCommandString;
        targetVelocityCommand.setZero();
        filteredVelocityCommand.setZero();
        teleopState.resetAxes();
        teleopState.holdPositionActive = true;
        teleopState.stabilizeModeActive = false;
        gamepadMotionRequiresRecenter = true;
        showVelocityModeHelp("Switched gait to '" + gaitCommandString + "'.");
      } else if (isMotionKey(key)) {
        updateTeleopAxisFromKeyboard(teleopState, key);
        activeMotionSource = MotionInputSource::Keyboard;
        keyboardMotionKeyProcessed = true;
        if (!teleopState.stabilizeModeActive) {
          ocs2::vector_t velocityCommand =
              composeVelocityCommand(teleopState, linearSpeed, lateralSpeed, yawSpeed, commandAxisScales);
          if (activeGaitCommand == "stance") {
            bool wasClamped = false;
            velocityCommand = clampVelocityCommandForStance(velocityCommand, wasClamped);
            if (wasClamped) {
              showVelocityModeHelp("Stance velocity clamp active.");
            }
          }
          targetVelocityCommand = velocityCommand;
          teleopState.holdPositionActive = false;
        } else {
          targetVelocityCommand.setZero();
          filteredVelocityCommand.setZero();
          teleopState.holdPositionActive = true;
        }
      } else if (key == 'y') {
        targetVelocityCommand.setZero();
        filteredVelocityCommand.setZero();
        teleopState.stabilizeModeActive = true;
        teleopState.holdPositionActive = true;
        gamepadMotionRequiresRecenter = true;
        showVelocityModeHelp("Stabilize mode active. Reference x/y follows current state.");
      } else if (key == 't') {
        teleopState.stabilizeModeActive = false;
        targetVelocityCommand =
            composeVelocityCommand(teleopState, linearSpeed, lateralSpeed, yawSpeed, commandAxisScales);
        if (targetVelocityCommand.isZero(1e-6)) {
          filteredVelocityCommand.setZero();
          teleopState.holdPositionActive = true;
        } else {
          teleopState.holdPositionActive = false;
        }
        showVelocityModeHelp("Walking mode resumed inside velocity mode.");
      } else if (key == 'o') {
        const ocs2::scalar_t desiredHeight = targetPoseCommand.adjustDesiredHeight(heightStep);
        if (teleopState.holdPositionActive) {
          targetPoseCommand.publishHoldPositionCommand(false);
        }
        showVelocityModeHelp("Desired height: " + std::to_string(desiredHeight));
      } else if (key == 'l') {
        const ocs2::scalar_t desiredHeight = targetPoseCommand.adjustDesiredHeight(-heightStep);
        if (teleopState.holdPositionActive) {
          targetPoseCommand.publishHoldPositionCommand(false);
        }
        showVelocityModeHelp("Desired height: " + std::to_string(desiredHeight));
      } else if (key == '+' || key == '=') {
        linearSpeed += linearStep;
        lateralSpeed += linearStep;
        yawSpeed += yawStep;
        showVelocityModeHelp("Speeds updated -> linear: " + std::to_string(linearSpeed) + " lateral: " +
                             std::to_string(lateralSpeed) + " yaw: " + std::to_string(yawSpeed));
      } else if (key == '-' || key == '_') {
        linearSpeed = std::max(minLinearSpeed, linearSpeed - linearStep);
        lateralSpeed = std::max(minLinearSpeed, lateralSpeed - linearStep);
        yawSpeed = std::max(minYawSpeed, yawSpeed - yawStep);
        showVelocityModeHelp("Speeds updated -> linear: " + std::to_string(linearSpeed) + " lateral: " +
                             std::to_string(lateralSpeed) + " yaw: " + std::to_string(yawSpeed));
      } else if (key == 'r') {
        if (controllerType == ControllerType::Rl) {
          showVelocityModeHelp("RL mode uses 1 for home hold and 2 for policy activation.");
        } else {
          targetVelocityCommand.setZero();
          filteredVelocityCommand.setZero();
          std::string stanceCommand = "stance";
          gaitCommand.publishKeyboardCommand(stanceCommand);
          activeGaitCommand = "stance";
          targetPoseCommand.publishHoldPositionCommand();
          teleopState.resetAxes();
          teleopState.holdPositionActive = true;
          teleopState.stabilizeModeActive = false;
          gamepadMotionRequiresRecenter = true;
          showVelocityModeHelp("Switched to stance. Motion stays enabled with stance safety limits.");
        }
      } else if (key == 'z') {
        if (controlState != ControlState::Mpc) {
          showVelocityModeHelp("Sit-down is only available while MPC is active in stance.");
        } else if (activeGaitCommand != "stance") {
          showVelocityModeHelp("Sit-down requires stance first. Press r to switch to stance.");
        } else {
          publishEmergencyOverrideCommand(emergencyOverridePublisher, ControlCommand::SitDown);
          targetVelocityCommand.setZero();
          filteredVelocityCommand.setZero();
          teleopState.resetAxes();
          teleopState.holdPositionActive = true;
          teleopState.stabilizeModeActive = false;
          gamepadMotionRequiresRecenter = true;
          targetPoseCommand.publishHoldPositionCommand(false);
          showVelocityModeHelp("Sit-down requested. MPC will hand over to strong-PD pose control.");
        }
      } else if (key == 'c') {
        showVelocityModeHelp("Zero torque is only available after sit-down or from a safe non-MPC mode.");
      } else if (key == 'g') {
        targetVelocityCommand.setZero();
        filteredVelocityCommand.setZero();
        targetPoseCommand.publishHoldPositionCommand();
        teleopState.resetAxes();
        teleopState.holdPositionActive = true;
        teleopState.stabilizeModeActive = false;
        currentMode = UserCommandMode::Goal;
        std::cout << "\nExited velocity mode. Back to goal mode.\n\n";
        break;
      }
    }

    gamepadInput.setStabilizeToggleState(teleopState.stabilizeModeActive);
    if (gamepadMotionRequiresRecenter) {
      if (!gamepadInput.hasActiveMotionInput()) {
        gamepadMotionRequiresRecenter = false;
      }
    } else if (!keyboardMotionKeyProcessed &&
               (gamepadInput.hasActiveMotionInput() || activeMotionSource == MotionInputSource::Gamepad)) {
      activeMotionSource = MotionInputSource::Gamepad;
      const ocs2::vector_t gamepadAxes = gamepadInput.currentMotionAxes();
      teleopState.forwardAxis = gamepadAxes(0);
      teleopState.lateralAxis = gamepadAxes(1);
      teleopState.yawAxis = gamepadAxes(2);
    }

    const auto now = std::chrono::steady_clock::now();
    const ocs2::scalar_t dt = std::clamp(
        static_cast<ocs2::scalar_t>(std::chrono::duration<double>(now - lastVelocityUpdateTime).count()),
        ocs2::scalar_t(0.0), ocs2::scalar_t(0.1));
    lastVelocityUpdateTime = now;

    if (teleopState.stabilizeModeActive) {
      targetVelocityCommand.setZero();
      filteredVelocityCommand.setZero();
      teleopState.holdPositionActive = true;
    } else {
      ocs2::vector_t velocityCommand =
          composeVelocityCommand(teleopState, linearSpeed, lateralSpeed, yawSpeed, commandAxisScales);

      if (activeGaitCommand == "stance") {
        bool wasClamped = false;
        velocityCommand = clampVelocityCommandForStance(velocityCommand, wasClamped);
      }

      targetVelocityCommand = velocityCommand;
      filteredVelocityCommand =
          stepTowardVelocityCommand(filteredVelocityCommand, targetVelocityCommand, accelerationLimits, dt);

      if (targetVelocityCommand.isZero(1e-6) && filteredVelocityCommand.isZero(1e-4)) {
        filteredVelocityCommand.setZero();
        teleopState.holdPositionActive = true;
      } else {
        teleopState.holdPositionActive = false;
      }
    }

    if (teleopState.holdPositionActive) {
      targetPoseCommand.publishHoldPositionCommand(false);
    } else {
      targetPoseCommand.publishVelocityCommand(filteredVelocityCommand, false);
    }
  }
}


int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
      robotName + "_user_command",
      rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true));

  // // Get node parameters-from launch
  const std::string gaitCommandFile =
      node->get_parameter("gaitCommandFile").as_string();
  std::cerr << "Loading gait file from launch: " << gaitCommandFile << std::endl;
  const std::string controlTypeParameter =
      node->has_parameter("controlType") ? node->get_parameter("controlType").as_string() : "mpc";
  const ControllerType controllerType = controllerTypeFromString(controlTypeParameter);
  std::cerr << "Using control type from launch: " << controlTypeParameter << std::endl;

  // const std::string gaitCommandFile = "/home/zhx/Desktop/zhx_legged_ocs2_ros2/src/legged_control/user_command/config/a1/gait.info";
  // std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;
  GaitKeyboardPublisher gaitCommand(node, gaitCommandFile, robotName, true);

  const ocs2::scalar_array_t relativeBaseLimit{100.0, 100.0, 0.2, 360.0};
  // const std::string referenceFile = "/home/zhx/Desktop/zhx_legged_ocs2_ros2/src/legged_control/user_command/config/a1/reference.info";
  const std::string referenceFile =
      node->get_parameter("referenceFile").as_string();
    std::cerr << "Loading reference file from launch: " << referenceFile << std::endl;
  boost::property_tree::ptree referenceInfoTree;
  boost::property_tree::read_info(referenceFile, referenceInfoTree);
  const double fallbackDisplacementVelocity =
      referenceInfoTree.get<double>("targetDisplacementVelocity", 0.20);
  const ocs2::scalar_t initialLinearSpeed =
      static_cast<ocs2::scalar_t>(referenceInfoTree.get<double>(
          "targetDisplacementVelocityForward", fallbackDisplacementVelocity));
  const ocs2::scalar_t initialLateralSpeed =
      static_cast<ocs2::scalar_t>(referenceInfoTree.get<double>(
          "targetDisplacementVelocityLateral", fallbackDisplacementVelocity));
  const ocs2::scalar_t initialYawSpeed =
      static_cast<ocs2::scalar_t>(referenceInfoTree.get<double>("targetRotationVelocity", 0.60));
  const ocs2::vector_t accelerationLimits =
      (ocs2::vector_t(3)
           << referenceInfoTree.get<double>("teleop.velocity_accel_limit_vx", 0.30),
              referenceInfoTree.get<double>("teleop.velocity_accel_limit_vy", 0.20),
              referenceInfoTree.get<double>("teleop.velocity_accel_limit_yaw", 0.60))
              .finished();
  const ocs2::vector_t commandAxisScales =
      (ocs2::vector_t(3)
           << referenceInfoTree.get<double>("command_axis.x", 1.0),
              referenceInfoTree.get<double>("command_axis.y", 1.0),
              referenceInfoTree.get<double>("command_axis.yaw", 1.0))
              .finished();
  GamepadConfig gamepadConfig;
  gamepadConfig.enabled = referenceInfoTree.get<bool>("teleop.gamepad.enabled", gamepadConfig.enabled);
  gamepadConfig.device = referenceInfoTree.get<std::string>("teleop.gamepad.device", gamepadConfig.device);
  gamepadConfig.axisDeadzone =
      referenceInfoTree.get<double>("teleop.gamepad.axisDeadzone", gamepadConfig.axisDeadzone);
  gamepadConfig.digitalPressThreshold =
      referenceInfoTree.get<double>("teleop.gamepad.digitalPressThreshold", gamepadConfig.digitalPressThreshold);
  gamepadConfig.axisLateral =
      referenceInfoTree.get<int>("teleop.gamepad.axisLateral", gamepadConfig.axisLateral);
  gamepadConfig.axisForward =
      referenceInfoTree.get<int>("teleop.gamepad.axisForward", gamepadConfig.axisForward);
  gamepadConfig.axisYaw =
      referenceInfoTree.get<int>("teleop.gamepad.axisYaw", gamepadConfig.axisYaw);
  gamepadConfig.dpadHorizontalAxis =
      referenceInfoTree.get<int>("teleop.gamepad.dpadHorizontalAxis", gamepadConfig.dpadHorizontalAxis);
  gamepadConfig.dpadVerticalAxis =
      referenceInfoTree.get<int>("teleop.gamepad.dpadVerticalAxis", gamepadConfig.dpadVerticalAxis);
  gamepadConfig.leftTriggerAxis =
      referenceInfoTree.get<int>("teleop.gamepad.leftTriggerAxis", gamepadConfig.leftTriggerAxis);
  gamepadConfig.rightTriggerAxis =
      referenceInfoTree.get<int>("teleop.gamepad.rightTriggerAxis", gamepadConfig.rightTriggerAxis);
  gamepadConfig.buttonB =
      referenceInfoTree.get<int>("teleop.gamepad.buttonB", gamepadConfig.buttonB);
  gamepadConfig.buttonX =
      referenceInfoTree.get<int>("teleop.gamepad.buttonX", gamepadConfig.buttonX);
  gamepadConfig.buttonY =
      referenceInfoTree.get<int>("teleop.gamepad.buttonY", gamepadConfig.buttonY);
  gamepadConfig.buttonLb =
      referenceInfoTree.get<int>("teleop.gamepad.buttonLb", gamepadConfig.buttonLb);
  gamepadConfig.buttonRb =
      referenceInfoTree.get<int>("teleop.gamepad.buttonRb", gamepadConfig.buttonRb);
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(node, robotName, relativeBaseLimit, referenceFile);
  auto emergencyOverridePublisher =
      node->create_publisher<std_msgs::msg::Int32>(robotName + "_emergency_override", 1);
  UserCommandMode currentMode = UserCommandMode::Goal;
  std::string activeGaitCommand = "stance";
  ControlState controlState = ControlState::Hold;
  auto emergencyOverrideStateSubscriber =
      node->create_subscription<std_msgs::msg::Int32>(
          robotName + "_emergency_override_state", 1,
          [&controlState](const std_msgs::msg::Int32::SharedPtr msg) {
            controlState = static_cast<ControlState>(msg->data);
          });
  (void)emergencyOverrideStateSubscriber;

  const std::string commadMsg =
    controllerType == ControllerType::Rl
        ? "Current mode starts as 'goal'.\n"
          "RL control starts in policyDefaultJointState hold. Enter 'mode:vel' and press '2' to activate RL.\n"
          "Enter 'mode:goal' or 'mode:vel' to switch modes,\n"
          "Enter 'gait:xxx' for the desired gait,\n"
          "Enter 'gait:list' for the list of available gaits,\n"
          "In goal mode use 'goal:x y z yaw_deg' for relative pose commands,\n"
          "In vel mode the node enters keyboard teleop with optional gamepad support,\n"
          "Use 'hold:' to hold the current pose.\n"
        : "Current mode starts as 'goal'.\n"
          "System starts latched in E-stop. Enter 'mode:vel' and press '1' to enable MPC stance.\n"
          "Enter 'mode:goal' or 'mode:vel' to switch modes,\n"
          "Enter 'gait:xxx' for the desired gait,\n"
          "Enter 'gait:list' for the list of available gaits,\n"
          "In goal mode use 'goal:x y z yaw_deg' for relative pose commands,\n"
          "In vel mode the node enters keyboard teleop with optional gamepad support,\n"
          "Use 'hold:' to hold the current pose.\n";
    
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (currentMode == UserCommandMode::Velocity) {
      try {
        runVelocityKeyboardMode(targetPoseCommand, gaitCommand, currentMode, activeGaitCommand,
                                initialLinearSpeed, initialLateralSpeed, initialYawSpeed,
                                accelerationLimits, commandAxisScales, controllerType, gamepadConfig, node,
                                emergencyOverridePublisher, controlState);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Velocity keyboard mode failed: %s", e.what());
        currentMode = UserCommandMode::Goal;
      }
      continue;
    }

    std::cout << commadMsg << ": ";

    auto shouldTerminate = []() { return !rclcpp::ok(); };
    const auto commandLine = ocs2::getCommandLineString(shouldTerminate);

    std::vector<std::string> splitInput = splitByColon(commandLine);
    //gaitCommand.getKeyboardCommand();
    if (splitInput.size() == 2) {
      std::string commandType = splitInput[0];  
      std::string commandValue = trimLeft(splitInput[1]);
      commandType = toLower(trimLeft(commandType));
      //std::string commandValue = splitInput[1]; 
      RCLCPP_INFO(node->get_logger(), "Command Type: %s", commandType.c_str());
      RCLCPP_INFO(node->get_logger(), "Command Value: %s", commandValue.c_str());
      // std::cout << "Command Type: " << commandType << std::endl;
      // std::cout << "Command Value: " << commandValue << std::endl;
      try {
        if (commandType == "mode") {
          const std::string modeValue = toLower(trimLeft(commandValue));
          if (modeValue == "goal") {
            currentMode = UserCommandMode::Goal;
          } else if (modeValue == "vel") {
            currentMode = UserCommandMode::Velocity;
          } else {
            RCLCPP_WARN(node->get_logger(), "Unknown mode '%s'. Use 'goal' or 'vel'.", modeValue.c_str());
            std::cout << std::endl;
            continue;
          }
          RCLCPP_INFO(node->get_logger(), "Switched user command mode to '%s'.", modeToString(currentMode).c_str());
          std::cout << "Mode switched to '" << modeToString(currentMode) << "'.\n\n";
        }
        else if (commandType == "gait")
        {
          activeGaitCommand = toLower(commandValue);
          gaitCommand.publishKeyboardCommand(commandValue);
        }
        else if (commandType == "goal")
        {
          if (currentMode != UserCommandMode::Goal) {
            RCLCPP_WARN(node->get_logger(), "Goal commands are only accepted in mode:goal.");
            std::cout << std::endl;
            continue;
          }
          ocs2::vector_t goalCommand = parseNumericCommand(commandValue, 4);
          applyAxisScalesToGoalCommand(goalCommand, commandAxisScales);
          if (activeGaitCommand == "stance") {
            bool wasClamped = false;
            goalCommand = clampGoalCommandForStance(goalCommand, wasClamped);
            if (wasClamped) {
              std::cout << "Stance target limit applied for safety.\n";
            }
          }
          targetPoseCommand.publishGoalCommand(goalCommand);
        }
        else if (commandType == "vel")
        {
          RCLCPP_WARN(node->get_logger(), "Typed 'vel:' commands are replaced by keyboard teleop inside mode:vel.");
          std::cout << std::endl;
        }
        else if (commandType == "hold")
        {
          targetPoseCommand.publishHoldPositionCommand();
        }
        else{
          RCLCPP_WARN(node->get_logger(), "Invalid command. Please use correct command.");
          std::cout << std::endl;
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(), "Failed to parse command: %s", e.what());
        std::cout << std::endl;
      }
    }
    else 
    {
      RCLCPP_WARN(node->get_logger(), "Invalid input format. Please use the format 'command:value'.");
      std::cout << std::endl;
    }
  }
  // Successful exit
  //std::cerr << "Succeed " << std::endl;
  return 0;
}
