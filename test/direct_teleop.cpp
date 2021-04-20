#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <controller_manager_msgs/SwitchController.h>


class DirectTeleop
{
    public:
        DirectTeleop(const std::string& name);
        ~DirectTeleop();
        void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);

    private:
        std::string node_name_;

        ros::Subscriber twist_sub_;
        ros::Publisher twist_pub_;

        ros::ServiceClient switch_client_;
};

DirectTeleop::DirectTeleop(const std::string& name) : node_name_(name)
{
    ros::NodeHandle nh; // Create a node handle and start the node
    ros::NodeHandle nh_priv("~");

    std::string cmd_sub_topic;
    std::string cmd_pub_topic;
    nh_priv.param<std::string>("cmd_sub_topic", cmd_sub_topic, std::string("/cmd_vel"));
    nh_priv.param<std::string>("cmd_pub_topic", cmd_pub_topic, std::string("/cmd_vel_stamped"));

    twist_sub_ = nh.subscribe<geometry_msgs::Twist>(cmd_sub_topic.c_str(), 1, &DirectTeleop::twistCallback, this);
    twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>(cmd_pub_topic, 10);

    switch_client_ = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

    controller_manager_msgs::SwitchController switch_srv;
    switch_srv.request.start_controllers.push_back("joint_group_position_controller");
    switch_srv.request.stop_controllers.push_back("arm_controller");
    switch_srv.request.strictness = switch_srv.request.STRICT;
    if (switch_client_.call(switch_srv))
    {
        ROS_INFO("Successfully switched controllers!");
    }
    else
    {
        ROS_ERROR("Failed to call service switch_controller");
    }
}

DirectTeleop::~DirectTeleop()
{
    // Switch back on destruction
    controller_manager_msgs::SwitchController switch_srv;
    switch_srv.request.start_controllers.push_back("arm_controller");
    switch_srv.request.stop_controllers.push_back("joint_group_position_controller");
    switch_srv.request.strictness = switch_srv.request.STRICT;
    if (switch_client_.call(switch_srv))
    {
        ROS_INFO("Successfully switched back!");
    }
    else
    {
        ROS_ERROR("Failed to revert service switch_controller");
    }
}

void DirectTeleop::twistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{ 
    geometry_msgs::TwistStamped twistStamped;
    twistStamped.header.stamp = ros::Time::now();
    twistStamped.header.frame_id = "world";
    twistStamped.twist = *twist;
    // Publish the stamped twist command
    twist_pub_.publish(twistStamped);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "direct_teleop");

    ROS_INFO_STREAM("Initialiasing the direct_teleop node");
    DirectTeleop teleop("direct_teleop");

    // Spin and leave work for callbacks
    ros::spin();

    return 0;
}