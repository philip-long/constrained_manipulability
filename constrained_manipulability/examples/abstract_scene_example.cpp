#include <chrono>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>

#include <rclcpp/rclcpp.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>

#include "constrained_manipulability/constrained_manipulability_utils.hpp"
#include "constrained_manipulability_interfaces/srv/add_remove_collision_solid.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("abstract_scene_example");

    // Declare parameters with default values
    node->declare_parameter<std::string>("object_primitives", "");
    node->declare_parameter<std::vector<std::string>>("object_dimensions", std::vector<std::string>{});
    node->declare_parameter<std::vector<std::string>>("object_poses", std::vector<std::string>{});

    // Get parameters
    std::string obj_primitives_str = node->get_parameter("object_primitives").as_string();
    std::vector<std::string> obj_dimensions_str = node->get_parameter("object_dimensions").as_string_array();
    std::vector<std::string> obj_poses_str = node->get_parameter("object_poses").as_string_array();

    // Parse the strings into vectors
    std::vector<int> obj_primitives = constrained_manipulability::parseVector<int>(obj_primitives_str);
    std::vector<std::vector<double>> obj_dimensions = constrained_manipulability::parseNestedVector<double>(obj_dimensions_str);
    std::vector<std::vector<double>> obj_poses = constrained_manipulability::parseNestedVector<double>(obj_poses_str);

    // Get collision shapes
    std::vector<shape_msgs::msg::SolidPrimitive> shapes_in;
    constrained_manipulability::TransformVector shape_poses;
    constrained_manipulability::getCollisionShapes(obj_primitives,
                                                   obj_dimensions,
                                                   obj_poses,
                                                   shapes_in,
                                                   shape_poses);

    // Add primitives to collision world
    auto client = node->create_client<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid>("add_remove_collision_solid");

    // Wait for the service to be available
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the 'add_remove_collision_solid' service. Exiting.");
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    // Create a request for each object
    for (unsigned int i = 0; i < obj_primitives.size(); ++i)
    {
        auto request = std::make_shared<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid::Request>();
        request->solid = shapes_in[i];
        request->pose = tf2::toMsg(shape_poses[i]);
        request->object_id = i;
        request->remove = false;

        // Call the service
        auto result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Collision object successfully added/removed.");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to add/remove collision object.");
        }
    }

    rclcpp::shutdown();

    return 0;
}