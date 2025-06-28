#include <rclcpp/rclcpp.hpp>


class NodeBase : public rclcpp::Node {

public:
    NodeBase(const std::string &name) : rclcpp::Node(name) {}
};


// ros2 pkg create zjros2 --dependencies rclcpp
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NodeBase>("demo");
  RCLCPP_INFO(node->get_logger(), "Successful initialization.");

  rclcpp::spin(node);
  // rclcpp::shutdown();
  return 0;
}
