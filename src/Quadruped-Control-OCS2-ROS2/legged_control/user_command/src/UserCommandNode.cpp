#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
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

enum class UserCommandMode {
  Goal,
  Velocity
};

std::string modeToString(UserCommandMode mode) {
  return mode == UserCommandMode::Goal ? "goal" : "vel";
}

struct TeleopCommandState {
  ocs2::scalar_t forwardAxis = 0.0;
  ocs2::scalar_t lateralAxis = 0.0;
  ocs2::scalar_t yawAxis = 0.0;
  bool holdPositionActive = true;
  std::chrono::steady_clock::time_point forwardUpdatedAt;
  std::chrono::steady_clock::time_point lateralUpdatedAt;
  std::chrono::steady_clock::time_point yawUpdatedAt;

  void resetAxes(const std::chrono::steady_clock::time_point& inactiveTime) {
    forwardAxis = 0.0;
    lateralAxis = 0.0;
    yawAxis = 0.0;
    forwardUpdatedAt = inactiveTime;
    lateralUpdatedAt = inactiveTime;
    yawUpdatedAt = inactiveTime;
  }
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
      return "dynamic_walk";
    case '5':
      return "pawup";
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

void printVelocityModeHelp() {
  std::cout
      << "\nVelocity mode keyboard control:\n"
      << "  w/s : forward/backward\n"
      << "  a/d : left/right strafe\n"
      << "  q/e : yaw left/right\n"
      << "  1 : stance\n"
      << "  2 : standing_trot\n"
      << "  3 : flying_trot\n"
      << "  4 : dynamic_walk\n"
      << "  5 : pawup\n"
      << "  o/l : raise/lower desired height slowly\n"
      << "  +/- : increase/decrease speeds\n"
      << "  space : switch to stance with safe clamped motion\n"
      << "  g : exit velocity mode back to goal mode\n"
      << "Keyboard currently drives full-scale axes. The teleop state is analog-ready for future controller support.\n\n";
}

void updateTeleopAxisFromKeyboard(TeleopCommandState& teleopState, char key,
                                  const std::chrono::steady_clock::time_point& now) {
  const ocs2::vector_t axis = normalizedAxisFromKey(key);
  if (std::abs(axis(0)) > 1e-6) {
    teleopState.forwardAxis = axis(0);
    teleopState.forwardUpdatedAt = now;
  }
  if (std::abs(axis(1)) > 1e-6) {
    teleopState.lateralAxis = axis(1);
    teleopState.lateralUpdatedAt = now;
  }
  if (std::abs(axis(2)) > 1e-6) {
    teleopState.yawAxis = axis(2);
    teleopState.yawUpdatedAt = now;
  }
}

void decayTeleopAxes(TeleopCommandState& teleopState, const std::chrono::steady_clock::time_point& now,
                     const std::chrono::milliseconds& axisTimeout) {
  if (now - teleopState.forwardUpdatedAt > axisTimeout) {
    teleopState.forwardAxis = 0.0;
  }
  if (now - teleopState.lateralUpdatedAt > axisTimeout) {
    teleopState.lateralAxis = 0.0;
  }
  if (now - teleopState.yawUpdatedAt > axisTimeout) {
    teleopState.yawAxis = 0.0;
  }
}

ocs2::vector_t composeVelocityCommand(const TeleopCommandState& teleopState, ocs2::scalar_t linearSpeed,
                                      ocs2::scalar_t lateralSpeed, ocs2::scalar_t yawSpeed) {
  ocs2::vector_t velocityCommand = ocs2::vector_t::Zero(3);
  velocityCommand(0) = teleopState.forwardAxis * linearSpeed;
  velocityCommand(1) = teleopState.lateralAxis * lateralSpeed;
  velocityCommand(2) = teleopState.yawAxis * yawSpeed;
  return velocityCommand;
}

void runVelocityKeyboardMode(TargetTrajectoriesKeyboardPublisher& targetPoseCommand,
                             GaitKeyboardPublisher& gaitCommand,
                             UserCommandMode& currentMode,
                             std::string& activeGaitCommand) {
  printVelocityModeHelp();

  ScopedRawTerminalMode rawTerminalMode;

  ocs2::scalar_t linearSpeed = 0.2;
  ocs2::scalar_t lateralSpeed = 0.15;
  ocs2::scalar_t yawSpeed = 0.6;
  const ocs2::scalar_t linearStep = 0.05;
  const ocs2::scalar_t yawStep = 0.1;
  const ocs2::scalar_t heightStep = 0.005;
  const ocs2::scalar_t minLinearSpeed = 0.05;
  const ocs2::scalar_t minYawSpeed = 0.1;
  ocs2::vector_t currentVelocityCommand = ocs2::vector_t::Zero(3);
  const auto inactiveTime = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  const auto axisTimeout = std::chrono::milliseconds(220);
  TeleopCommandState teleopState;
  teleopState.resetAxes(inactiveTime);

  while (rclcpp::ok() && currentMode == UserCommandMode::Velocity) {
    const char rawKey = readSingleKey(std::chrono::milliseconds(50));
    const char key = rawKey == '\0' ? '\0' : static_cast<char>(std::tolower(static_cast<unsigned char>(rawKey)));
    const auto now = std::chrono::steady_clock::now();

    if (key != '\0') {
      const std::string gaitCommandString = gaitCommandFromKey(key);
      if (!gaitCommandString.empty()) {
        std::string gaitCommandValue = gaitCommandString;
        gaitCommand.publishKeyboardCommand(gaitCommandValue);
        activeGaitCommand = gaitCommandString;
        currentVelocityCommand.setZero();
        teleopState.resetAxes(inactiveTime);
        teleopState.holdPositionActive = true;
        std::cout << "Switched gait to '" << gaitCommandString << "'.\n";
      } else if (isMotionKey(key)) {
        updateTeleopAxisFromKeyboard(teleopState, key, now);
        ocs2::vector_t velocityCommand = composeVelocityCommand(teleopState, linearSpeed, lateralSpeed, yawSpeed);
        if (activeGaitCommand == "stance") {
          bool wasClamped = false;
          velocityCommand = clampVelocityCommandForStance(velocityCommand, wasClamped);
          if (wasClamped) {
            std::cout << "\rStance velocity clamp active.                      " << std::flush;
          }
        }
        currentVelocityCommand = velocityCommand;
        teleopState.holdPositionActive = false;
      } else if (key == 'o') {
        const ocs2::scalar_t desiredHeight = targetPoseCommand.adjustDesiredHeight(heightStep);
        if (teleopState.holdPositionActive) {
          targetPoseCommand.publishHoldPositionCommand(false);
        }
        std::cout << "\rDesired height: " << desiredHeight << "            " << std::flush;
      } else if (key == 'l') {
        const ocs2::scalar_t desiredHeight = targetPoseCommand.adjustDesiredHeight(-heightStep);
        if (teleopState.holdPositionActive) {
          targetPoseCommand.publishHoldPositionCommand(false);
        }
        std::cout << "\rDesired height: " << desiredHeight << "            " << std::flush;
      } else if (key == '+' || key == '=') {
        linearSpeed += linearStep;
        lateralSpeed += linearStep;
        yawSpeed += yawStep;
        std::cout << "\rSpeeds updated -> linear: " << linearSpeed << " lateral: " << lateralSpeed << " yaw: " << yawSpeed
                  << "            " << std::flush;
      } else if (key == '-' || key == '_') {
        linearSpeed = std::max(minLinearSpeed, linearSpeed - linearStep);
        lateralSpeed = std::max(minLinearSpeed, lateralSpeed - linearStep);
        yawSpeed = std::max(minYawSpeed, yawSpeed - yawStep);
        std::cout << "\rSpeeds updated -> linear: " << linearSpeed << " lateral: " << lateralSpeed << " yaw: " << yawSpeed
                  << "            " << std::flush;
      } else if (key == ' ') {
        currentVelocityCommand.setZero();
        std::string stanceCommand = "stance";
        gaitCommand.publishKeyboardCommand(stanceCommand);
        activeGaitCommand = "stance";
        targetPoseCommand.publishHoldPositionCommand();
        teleopState.resetAxes(inactiveTime);
        teleopState.holdPositionActive = true;
        std::cout << "Switched to stance. Motion stays enabled with stance safety limits.\n";
      } else if (key == 'g') {
        currentVelocityCommand.setZero();
        targetPoseCommand.publishHoldPositionCommand();
        teleopState.resetAxes(inactiveTime);
        teleopState.holdPositionActive = true;
        currentMode = UserCommandMode::Goal;
        std::cout << "\nExited velocity mode. Back to goal mode.\n\n";
        break;
      }
    }

    decayTeleopAxes(teleopState, now, axisTimeout);
    ocs2::vector_t velocityCommand = composeVelocityCommand(teleopState, linearSpeed, lateralSpeed, yawSpeed);

    if (velocityCommand.isZero(1e-6)) {
      currentVelocityCommand.setZero();
      teleopState.holdPositionActive = true;
    } else {
      if (activeGaitCommand == "stance") {
        bool wasClamped = false;
        velocityCommand = clampVelocityCommandForStance(velocityCommand, wasClamped);
      }
      currentVelocityCommand = velocityCommand;
      teleopState.holdPositionActive = false;
    }

    if (teleopState.holdPositionActive) {
      targetPoseCommand.publishHoldPositionCommand(false);
    } else {
      targetPoseCommand.publishVelocityCommand(currentVelocityCommand, false);
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

  // const std::string gaitCommandFile = "/home/zhx/Desktop/zhx_legged_ocs2_ros2/src/legged_control/user_command/config/a1/gait.info";
  // std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;
  GaitKeyboardPublisher gaitCommand(node, gaitCommandFile, robotName, true);

  const ocs2::scalar_array_t relativeBaseLimit{100.0, 100.0, 0.2, 360.0};
  // const std::string referenceFile = "/home/zhx/Desktop/zhx_legged_ocs2_ros2/src/legged_control/user_command/config/a1/reference.info";
  const std::string referenceFile =
      node->get_parameter("referenceFile").as_string();
    std::cerr << "Loading reference file from launch: " << referenceFile << std::endl;
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(node, robotName, relativeBaseLimit, referenceFile);
  UserCommandMode currentMode = UserCommandMode::Goal;
  std::string activeGaitCommand = "stance";

  const std::string commadMsg =
    "Current mode starts as 'goal'.\n"
    "Enter 'mode:goal' or 'mode:vel' to switch modes,\n"
    "Enter 'gait:xxx' for the desired gait,\n"
    "Enter 'gait:list' for the list of available gaits,\n"
    "In goal mode use 'goal:x y z yaw_deg' for relative pose commands,\n"
    "In vel mode the node enters keyboard teleop,\n"
    "Use 'hold:' to hold the current pose.\n";
    
  while (rclcpp::ok()) {
    if (currentMode == UserCommandMode::Velocity) {
      try {
        runVelocityKeyboardMode(targetPoseCommand, gaitCommand, currentMode, activeGaitCommand);
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
