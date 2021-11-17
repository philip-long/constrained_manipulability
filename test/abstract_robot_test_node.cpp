#include <constrained_manipulability/constrained_manipulability.h>

sensor_msgs::JointState joint_state;
bool joint_state_received(false);

void jointSensorCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state = *msg;
    joint_state_received = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "constrained_manipulability");
    
    ros::NodeHandle nh; // Create a node handle and start the node
    constrained_manipulability::PolytopeVolume polytope_volumes;

    ros::Subscriber joint_sub = nh.subscribe("/joint_states",
                                             1, &jointSensorCallback);

    ros::Publisher vol_pub = nh.advertise<constrained_manipulability::PolytopeVolume>("constrained_manipulability/polytope_volumes", 1);
    std::string root, tip;
    std::vector<int> object_primitive;
    std::vector<std::vector<double>> obj_dimensions;
    std::vector<std::vector<double>> obj_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes_in;
    TransformVector shapes_pose;
    FCLObjectSet objects;
    bool show_mp, show_cmp;

    utility_functions::getParameter("~/root", root);
    utility_functions::getParameter("~/tip", tip);
    utility_functions::getParameter("~/show_mp", show_mp);
    utility_functions::getParameter("~/show_cmp", show_cmp);
    utility_functions::getVectorParam("~/object_primitive", object_primitive);
    utility_functions::getVectorVectorParam("~/object_dimensions", obj_dimensions);
    utility_functions::getVectorVectorParam("~/object_poses", obj_poses);
    utility_functions::getCollisionShapes(object_primitive,
                                          obj_dimensions,
                                          obj_poses,
                                          shapes_in,
                                          shapes_pose);

    ConstrainedManipulability robot_polytope(nh, root, tip);

    bool use_static_functions(false);

    // TEST FOR STATIC FUNCTIONS
    KDL::Tree my_tree_;
    KDL::Chain chain_;
    urdf::Model model_;
    std::string robot_desc_string;

    if (use_static_functions)
    {
        nh.param("robot_description", robot_desc_string, std::string());
        model_.initParamWithNodeHandle("robot_description", nh);
        if (!kdl_parser::treeFromString(robot_desc_string, my_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
        }
        else
        {
            ROS_INFO("Success");
        }
        my_tree_.getChain(root,
                          tip,
                          chain_);
    }
    objects.resize(shapes_in.size());
    for (int i = 0; i < shapes_in.size(); ++i)
    {
        robot_polytope.addCollisionObject(shapes_in[i], shapes_pose[i], i);
        objects[i].object_shape = shapes_in[i];
        objects[i].object_transform = shapes_pose[i];
    }
    ros::Duration(2.0).sleep();
    robot_polytope.displayObjects();

    polytope_volumes.names.resize(4);
    polytope_volumes.names[0] = "allowable_motion_polytope";
    polytope_volumes.names[1] = "constrained_allowable_motion_polytope";
    polytope_volumes.names[2] = "velocity_polytope";
    polytope_volumes.names[3] = "constrained_velocity_polytope";

    polytope_volumes.volumes.resize(4);
    while (ros::ok())
    {
        if (joint_state_received == true)
        {
            joint_state_received = false;

            robot_polytope.checkCollision(joint_state);
            polytope_volumes.volumes[0] = robot_polytope.getAllowableMotionPolytope(joint_state,
                                                                                    show_mp,
                                                                                    {0.0, 0.0, 0.5, 0.0},
                                                                                    {0.0, 0.0, 1.0, 0.4});
            polytope_volumes.volumes[1] = robot_polytope.getConstrainedAllowableMotionPolytope(joint_state,
                                                                                               show_cmp,
                                                                                               {0.0, 0.0, 0.5, 0.0},
                                                                                               {1.0, 0.0, 0.0, 0.4});
            polytope_volumes.volumes[2] = robot_polytope.getVelocityPolytope(joint_state, false);
            polytope_volumes.volumes[3] = robot_polytope.getConstrainedVelocityPolytope(joint_state, false);

            if (use_static_functions)
            {

                Eigen::MatrixXd A;
                Eigen::VectorXd b;
                polytope_volumes.volumes[0] = ConstrainedManipulability::getAllowableMotionPolytope(chain_, model_, joint_state, A, b);
                polytope_volumes.volumes[1] = ConstrainedManipulability::getConstrainedAllowableMotionPolytope(chain_, model_, objects, joint_state, A, b);
                polytope_volumes.volumes[2] = ConstrainedManipulability::getVelocityPolytope(chain_, model_, joint_state, A, b);
                polytope_volumes.volumes[3] = ConstrainedManipulability::getConstrainedVelocityPolytope(chain_, model_, objects, joint_state, A, b);
            }
            vol_pub.publish(polytope_volumes);
        }
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
    return 0;
}
