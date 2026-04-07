#include "real_robot_bridge/BridgeNodeBase.h"
#include "real_robot_bridge/HardwareBackend.h"

namespace real_robot_bridge {

class RealBridgeNode final : public BridgeNodeBase {
 public:
  explicit RealBridgeNode(const rclcpp::NodeOptions& options)
      : BridgeNodeBase("bridge_real_node", options) {
    initializeBackend(std::make_unique<HardwareBackend>(*this));
  }
};

}  // namespace real_robot_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<real_robot_bridge::RealBridgeNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
