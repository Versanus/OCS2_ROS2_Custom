#pragma once

#include <string>

struct ControllerConfig {
  std::string robotName{"legged_robot"};
  std::string controlType{"mpc"};
  std::string taskFile;
  std::string urdfFile;
  std::string referenceFile;
  std::string simulatorFile;
  std::string rlConfigFile;
  std::string rlFeedbackJointStateTransform;
};
