#include "motion_control/controller/LeggedControllerRosInterface.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "legged_robot_controller",
      rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));

  LeggedControllerRosInterface controller(node);
  controller.launch();
  rclcpp::shutdown();

  return 0;
}
