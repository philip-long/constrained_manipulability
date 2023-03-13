#include <random>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Float32.h>

#include <robot_collision_checking/fcl_interface.h>

#include <constrained_manipulability/constrained_manipulability.hpp>

sensor_msgs::JointState joint_state;
std_msgs::Float32 lin_limit;
bool joint_state_received(false), lin_callback_received(false);

void jointSensorCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state = *msg;
    joint_state_received = true;
}

void linCallback(const std_msgs::Float32::ConstPtr &msg)
{
    lin_limit = *msg;
    lin_callback_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shrinking_polytope_test");
    std::srand(std::time(nullptr));

    ros::NodeHandle nh; // Create a node handle and start the node

    ros::Subscriber joint_sub = nh.subscribe("/joint_states",
                                             1, &jointSensorCallback);

    ros::Subscriber link_sub = nh.subscribe("/lin_limit",
                                            1, &linCallback);

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    std::string root, tip;
    std::vector<int> object_primitive;
    std::vector<std::vector<double>> obj_dimensions;
    std::vector<std::vector<double>> obj_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes_in;
    constrained_manipulability::TransformVector shapes_pose;
    robot_collision_checking::FCLObjectSet objects;
    bool show_mp, show_cmp, debug_statements;

    constrained_manipulability::getParameter("~/debug_statements", debug_statements);
    constrained_manipulability::getParameter("~/root", root);
    constrained_manipulability::getParameter("~/tip", tip);
    constrained_manipulability::getParameter("~/show_mp", show_mp);
    constrained_manipulability::getParameter("~/show_cmp", show_cmp);
    constrained_manipulability::getVectorParam("~/object_primitive", object_primitive);
    constrained_manipulability::getVectorVectorParam("~/object_dimensions", obj_dimensions);
    constrained_manipulability::getVectorVectorParam("~/object_poses", obj_poses);
    constrained_manipulability::getCollisionShapes(object_primitive,
                                                   obj_dimensions,
                                                   obj_poses,
                                                   shapes_in,
                                                   shapes_pose);

    constrained_manipulability::ConstrainedManipulability constrained_manip(nh, root, tip);

    ROS_INFO("Adding Objects");
    objects.resize(shapes_in.size());
    for (int i = 0; i < shapes_in.size(); ++i)
    {
        constrained_manip.addCollisionObject(shapes_in[i], shapes_pose[i], i);
        objects[i].object_shape = shapes_in[i];
        objects[i].object_transform = shapes_pose[i];
    }
    ros::Duration(2.0).sleep();
    ROS_INFO("Displaying Objects");
    constrained_manip.displayObjects();

    while (ros::ok())
    {

        if (joint_state_received == true)
        {
            joint_state_received = false;

            if (lin_callback_received == true)
            {
                lin_callback_received == false;
                constrained_manip.setLinearizationLimit(lin_limit.data);
            }
            constrained_manip.checkCollision(joint_state);
            constrained_manipulability::Polytope allowable_poly = constrained_manip.getAllowableMotionPolytope(
                joint_state,
                show_mp,
                {0.0, 0.0, 0.5, 0.0},
                {0.0, 0.0, 1.0, 0.4});
            constrained_manipulability::Polytope constrained_poly = constrained_manip.getConstrainedAllowableMotionPolytope(
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
