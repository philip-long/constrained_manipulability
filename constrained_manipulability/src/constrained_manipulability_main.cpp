#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "constrained_manipulability/constrained_manipulability.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;
  auto node = std::make_shared<constrained_manipulability::ConstrainedManipulability>(options);

  // Use a MultiThreadedExecutor to handle multiple callback groups
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();

  return 0;
}