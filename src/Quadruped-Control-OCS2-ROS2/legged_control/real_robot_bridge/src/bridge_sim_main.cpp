#include "real_robot_bridge/BridgeNodeBase.h"
#include "real_robot_bridge/MujocoBackend.h"

namespace real_robot_bridge {

class SimBridgeNode final : public BridgeNodeBase {
 public:
  explicit SimBridgeNode(const rclcpp::NodeOptions& options)
      : BridgeNodeBase("bridge_sim_node", options) {
    const auto xml_file = declare_parameter<std::string>("xmlFile", "");
    const auto simulator_file = declare_parameter<std::string>("simulatorFile", "");
    initializeBackend(std::make_unique<MujocoBackend>(xml_file, simulator_file));
  }
};

}  // namespace real_robot_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<real_robot_bridge::SimBridgeNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
