#include <chrono>
#include <vector>

#include <boost/thread/mutex.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "constrained_manipulability_interfaces/srv/get_sliced_polytope.hpp"

class SlicingPolytopeExample : public rclcpp::Node
{
    public:
        SlicingPolytopeExample() : Node("slicing_polytope_example") 
        {
            polytope_type_ = this->declare_parameter<int>("polytope_type", 2); // CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE
            plane_width_ = this->declare_parameter<double>("plane_width", 0.004);

            client_ = this->create_client<constrained_manipulability_interfaces::srv::GetSlicedPolytope>("get_sliced_polytope");
            
            // Wait for the service to be available
            while (!client_->wait_for_service(std::chrono::seconds(1))) 
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&SlicingPolytopeExample::jointStateCallback, this, std::placeholders::_1));

            request_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SlicingPolytopeExample::timerCallback, this));
        }

    private:
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
        {
            boost::mutex::scoped_lock lock(joint_state_mutex_);
            curr_joint_state_ = *msg;
        }
        
        void timerCallback() 
        {
            auto req = std::make_shared<constrained_manipulability_interfaces::srv::GetSlicedPolytope::Request>();
            req->polytope_type = polytope_type_;
            req->plane_width = plane_width_;
            joint_state_mutex_.lock();
            req->joint_state = curr_joint_state_;
            joint_state_mutex_.unlock();

            // Send requests for different slicing planes
            sendSlicedPolytopeRequest(req, 0);
            sendSlicedPolytopeRequest(req, 1);
            sendSlicedPolytopeRequest(req, 2);     
        }

        void sendSlicedPolytopeRequest(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetSlicedPolytope::Request> req, int plane)
        {
            req->slicing_plane = plane;

            using ServiceResponseFuture = rclcpp::Client<constrained_manipulability_interfaces::srv::GetSlicedPolytope>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future) {
                auto response = future.get();
                if (response->polytope.name != "invalid_polytope")
                {
                    RCLCPP_INFO(this->get_logger(), "Successfully computed the '%s'.", response->polytope.name.c_str());
                }
            };
            auto future_result = client_->async_send_request(req, response_received_callback);
        }
        
        // Private slicing properties
        int polytope_type_;
        double plane_width_;

        sensor_msgs::msg::JointState curr_joint_state_;
        boost::mutex joint_state_mutex_;

        // ROS interface members
        rclcpp::TimerBase::SharedPtr request_timer_;
        rclcpp::Client<constrained_manipulability_interfaces::srv::GetSlicedPolytope>::SharedPtr client_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlicingPolytopeExample>());
    rclcpp::shutdown();
    return 0;
}