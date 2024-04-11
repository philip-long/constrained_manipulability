#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "constrained_manipulability/constrained_manipulability.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<constrained_manipulability::ConstrainedManipulability>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}