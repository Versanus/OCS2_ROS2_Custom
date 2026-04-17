#pragma once

#include <memory>

#include "motion_control/controller/ControllerBackend.h"
#include "rclcpp/rclcpp.hpp"

class MPC_WBC_ROS_Interface;

class MpcBackend final : public ControllerBackend {
 public:
  explicit MpcBackend(const rclcpp::Node::SharedPtr& node);
  ~MpcBackend() override;

  bool configure(const ControllerConfig& config) override;
  void launch() override;
  const char* name() const override;

 private:
  rclcpp::Node::SharedPtr node_;
  ControllerConfig config_;
  std::unique_ptr<MPC_WBC_ROS_Interface> mpcInterface_;
};
