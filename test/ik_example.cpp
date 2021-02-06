#include <constrained_manipulability/constrained_manipulability.h>
#include <random>
#include "snoptProblem.hpp"

sensor_msgs::JointState joint_state;
bool joint_state_received ( false );

std::vector<double> q_reference_g; // reference joint position
Eigen::MatrixXd AHint_g; // Hyperplane constraints
Eigen::VectorXd bHint_g,dq_current (6); // Hyperplane constraints
Eigen::VectorXd desired_twist; 
Eigen::Matrix<double,6,6> J_global; // Global Jacobian




void jointSensorCallback ( const sensor_msgs::JointState::ConstPtr& msg ) {
    joint_state=*msg;
    joint_state_received=true;
}


void sampleJointStates(sensor_msgs::JointState current_state, std::vector<sensor_msgs::JointState>& sampled_state)
{
    double min_deviation=-0.1;
    double max_deviation=0.1;
    for ( auto& one_sample_state:sampled_state) {
        one_sample_state=current_state;                
        for ( auto& one_joint_position:one_sample_state.position) {
            one_joint_position+= (min_deviation + (double)(rand()) / ((double)(RAND_MAX/(max_deviation - min_deviation))) );
        }
    }
}



int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "ik_example" );
    std::srand(std::time(nullptr));
    ROS_INFO ( "FCL SAWYER" );
    ros::NodeHandle nh; // Create a node handle and start the node
    constrained_manipulability::PolytopeVolume polytope_volumes;


    ros::Subscriber  joint_sub= nh.subscribe ( "/joint_states",
                                1, &jointSensorCallback );

    ros::Publisher vol_pub=nh.advertise<constrained_manipulability::PolytopeVolume>
                        ( "constrained_manipulability/polytope_volumes",1 );
    std::string root,tip;
    std::vector<int> object_primitive;
    std::vector<std::vector<double>> obj_dimensions;
    std::vector<std::vector<double>> obj_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes_in;
    TransformVector shapes_pose;
    FCLObjectSet objects;
    bool show_mp,show_cmp;

    utility_functions::getParameter ( "~/root",root );
    utility_functions::getParameter ( "~/tip",tip );
    utility_functions::getParameter ( "~/show_mp",show_mp );
    utility_functions::getParameter ( "~/show_cmp",show_cmp );
    utility_functions::getVectorParam ( "~/object_primitive",object_primitive );
    utility_functions::getVectorVectorParam ( "~/object_dimensions",obj_dimensions );
    utility_functions::getVectorVectorParam ( "~/object_poses",obj_poses );
    utility_functions::getCollisionShapes ( object_primitive,
                                            obj_dimensions,
                                            obj_poses,
                                            shapes_in,
                                            shapes_pose );




    ConstrainedManipulability robot_polytope ( nh,root,tip );

    objects.resize ( shapes_in.size() );
    for ( int i = 0; i < shapes_in.size(); ++i ) {
        robot_polytope.addCollisionObject ( shapes_in[i],shapes_pose[i],i );
        objects[i].object_shape=shapes_in[i];
        objects[i].object_transform=shapes_pose[i];
    }
    ros::Duration ( 2.0 ).sleep();
    robot_polytope.displayObjects();




    polytope_volumes.names.resize ( 4 );
    polytope_volumes.names[0]= "AllowableMotion";
    polytope_volumes.names[1]="ConstrainedAllowableMotion";
    polytope_volumes.names[2]="VelocityPolytope";
    polytope_volumes.names[3]="ConstrainedVelocityPolytope";

    polytope_volumes.volumes.resize ( 4 );
    
    std::vector<sensor_msgs::JointState> sampled_joint_state(5);
    
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
        
    while ( ros::ok() ) {
        
        if ( joint_state_received==true ) {
            joint_state_received=false;
        
            // This function simply finds random joint configurations in the neighborhood
            sampleJointStates(joint_state,sampled_joint_state);
            
            
            
            double color_a=0.0;
            // Here we cycle through the sampled joints and display the polytope for each one
            for ( auto& one_sample:sampled_joint_state)
            {
                color_a+=0.2;
                // unconstrained polytope
                robot_polytope.getAllowableMotionPolytope( one_sample,
                        show_mp,
                        {0.0,0.0,0.5,0.0},
                        {0.0,0.0,color_a,0.4} );
                // constrained polytope                
                robot_polytope.getConstrainedAllowableMotionPolytope( one_sample,
                                        AHrep,
                                        bhrep,
                                        show_mp,
                                        {0.0,0.0,0.5,0.0},
                                        {color_a,0.0,0.0,0.4} );

                // Here we do the SNOPT stuff, the question is whether we do it as a velocity or a 
                ros::spinOnce();
                ros::Duration ( 0.5 ).sleep();                       
            }
            
            

            //vol_pub.publish ( polytope_volumes );
        }
        ros::spinOnce();
        ros::Duration ( 0.001 ).sleep();
    }
    return 0;

}
