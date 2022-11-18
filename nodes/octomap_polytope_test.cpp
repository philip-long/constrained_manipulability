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
    ros::init(argc, argv, "octomap_polytope_test");
    std::srand(std::time(nullptr));

    ros::NodeHandle nh; // Create a node handle and start the node

    ros::Subscriber joint_sub = nh.subscribe("/joint_states",
                                             1, &jointSensorCallback);

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    std::string root, tip;
    bool show_mp, show_cmp, debug_statements;

    constrained_manipulability::getParameter("~/debug_statements", debug_statements);
    constrained_manipulability::getParameter("~/root", root);
    constrained_manipulability::getParameter("~/tip", tip);
    constrained_manipulability::getParameter("~/show_mp", show_mp);
    constrained_manipulability::getParameter("~/show_cmp", show_cmp);

    constrained_manipulability::ConstrainedManipulability constrained_manip(nh, root, tip);

    while (ros::ok())
    {

        if (joint_state_received == true)
        {
            joint_state_received = false;

            ROS_ERROR_COND(constrained_manip.checkCollision(joint_state), "Robot in collision");

            constrained_manipulability::Polytope allow_poly = constrained_manip.getAllowableMotionPolytope(
                joint_state,
                show_mp,
                {0.0, 0.0, 0.5, 0.0},
                {0.0, 0.0, 1.0, 0.4});
            constrained_manipulability::Polytope const_poly = constrained_manip.getConstrainedAllowableMotionPolytope(
                joint_state,
                show_cmp,
                {0.0, 0.0, 0.5, 0.0},
                {1.0, 0.0, 0.0, 0.4});
        }

        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
    return 0;
}
