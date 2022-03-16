#include <constrained_manipulability/constrained_manipulability.h>
#include <random>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

sensor_msgs::JointState joint_state;
std_msgs::Float32 lin_limit;
bool joint_state_received(false), lin_callback_received(false);

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
    constrained_manipulability::PolytopeVolume polytope_volumes;

    std::string root, tip;
    std::vector<int> object_primitive;
    std::vector<std::vector<double>> obj_dimensions;
    std::vector<std::vector<double>> obj_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes_in;
    TransformVector shapes_pose;
    FCLObjectSet objects;
    bool show_mp, show_cmp, set_rviz_wait, debug_statements;

    utility_functions::getParameter("~/debug_statements", debug_statements);
    utility_functions::getParameter("~/root", root);
    utility_functions::getParameter("~/tip", tip);
    utility_functions::getParameter("~/show_mp", show_mp);
    utility_functions::getParameter("~/show_cmp", show_cmp);
    utility_functions::getParameter("~/set_rviz_wait", set_rviz_wait);
    utility_functions::getVectorParam("~/object_primitive", object_primitive);
    utility_functions::getVectorVectorParam("~/object_dimensions", obj_dimensions);
    utility_functions::getVectorVectorParam("~/object_poses", obj_poses);
    utility_functions::getCollisionShapes(object_primitive,
                                          obj_dimensions,
                                          obj_poses,
                                          shapes_in,
                                          shapes_pose);

    ConstrainedManipulability robot_polytope(nh, root, tip);
    robot_polytope.setRvizWait(set_rviz_wait);

    ROS_INFO("Adding Objects");
    objects.resize(shapes_in.size());
    for (int i = 0; i < shapes_in.size(); ++i)
    {
        robot_polytope.addCollisionObject(shapes_in[i], shapes_pose[i], i);
        objects[i].object_shape = shapes_in[i];
        objects[i].object_transform = shapes_pose[i];
    }
    ros::Duration(2.0).sleep();
    ROS_INFO("Displaying Objects");
    robot_polytope.displayObjects();
    ROS_INFO("Finished display");

    polytope_volumes.names.resize(4);
    polytope_volumes.names[0] = "AllowableMotion";
    polytope_volumes.names[1] = "ConstrainedAllowableMotion";
    polytope_volumes.names[2] = "VelocityPolytope";
    polytope_volumes.names[3] = "ConstrainedVelocityPolytope";

    polytope_volumes.volumes.resize(4);

    ros::spin();
    return 0;
}
