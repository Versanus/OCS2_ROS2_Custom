#pragma once

#include <memory>

#include "motion_control/controller/ControllerBackend.h"
#include "motion_control/controller/ControllerConfig.h"
#include "rclcpp/rclcpp.hpp"

class LeggedControllerRosInterface {
 public:
  explicit LeggedControllerRosInterface(const rclcpp::Node::SharedPtr& node);

  void launch();

 private:
  ControllerConfig loadConfig() const;
  std::unique_ptr<ControllerBackend> createBackend(const ControllerConfig& config) const;

  rclcpp::Node::SharedPtr node_;
  ControllerConfig config_;
  std::unique_ptr<ControllerBackend> backend_;
};
