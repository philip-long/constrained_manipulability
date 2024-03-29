#include <random>
#include <string>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <constrained_manipulability/constrained_manipulability.hpp>

sensor_msgs::JointState joint_state;
bool joint_state_received(false);

void jointSensorCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state = *msg;
    joint_state_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "constrained_manipulability_server");
    std::srand(std::time(nullptr));

    ros::NodeHandle nh; // Create a node handle and start the node

    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 10, &jointSensorCallback);

    std::string root, tip, robot_desc;

    constrained_manipulability::getParameter("~/root", root);
    constrained_manipulability::getParameter("~/tip", tip);
    constrained_manipulability::getParameter("~/robot_desc", robot_desc);

    constrained_manipulability::ConstrainedManipulability constrained_manip(nh, root, tip, robot_desc);

    // Loop with 100 Hz rate
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if (joint_state_received == true)
        {
            joint_state_received = false;

            ROS_ERROR_COND(constrained_manip.checkCollision(joint_state), "Robot in collision");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
