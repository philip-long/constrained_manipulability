#include <constrained_manipulability/constrained_manipulability.h>
#include <random>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

sensor_msgs::JointState joint_state;
std_msgs::Float32 lin_limit;
bool joint_state_received ( false ),lin_callback_received(false);

void jointSensorCallback ( const sensor_msgs::JointState::ConstPtr& msg ) {
    joint_state=*msg;
    joint_state_received=true;
}

void linCallback ( const std_msgs::Float32::ConstPtr& msg ) {
    lin_limit=*msg;
    lin_callback_received=true;
}


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "slicing_polytope" );
    std::srand(std::time(nullptr));

    ros::NodeHandle nh; // Create a node handle and start the node
    constrained_manipulability::PolytopeVolume polytope_volumes;


    ros::Subscriber  joint_sub= nh.subscribe ( "/joint_states",
                                1, &jointSensorCallback );

    ros::Subscriber  link_sub= nh.subscribe ( "/lin_limit",
                                1, &linCallback );
    
    ros::Publisher vol_pub=nh.advertise<constrained_manipulability::PolytopeVolume>
                           ( "constrained_manipulability/polytope_volumes",1 );

    ros::Publisher joint_pub=nh.advertise<sensor_msgs::JointState>
                             ( "joint_states",1 );


    std::string root,tip;
    std::vector<int> object_primitive;
    std::vector<std::vector<double>> obj_dimensions;
    std::vector<std::vector<double>> obj_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes_in;
    TransformVector shapes_pose;
    FCLObjectSet objects;
    bool show_mp,show_cmp,debug_statements;

    utility_functions::getParameter ( "~/debug_statements",debug_statements );
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

      ROS_INFO ( "Adding Objects" );	
    objects.resize ( shapes_in.size() );
    for ( int i = 0; i < shapes_in.size(); ++i ) {
        robot_polytope.addCollisionObject ( shapes_in[i],shapes_pose[i],i );
        objects[i].object_shape=shapes_in[i];
        objects[i].object_transform=shapes_pose[i];
    }
    ros::Duration ( 2.0 ).sleep();
      ROS_INFO ( "Displaying Objects" );
    robot_polytope.displayObjects();

    Eigen::MatrixXd AHrep,Vset;
    Eigen::VectorXd bhrep;

    Eigen::Vector3d offset_position;

    polytope_volumes.names.resize ( 4 );
    polytope_volumes.names[0]= "AllowableMotion";
    polytope_volumes.names[1]="ConstrainedAllowableMotion";
    polytope_volumes.names[2]="VelocityPolytope";
    polytope_volumes.names[3]="ConstrainedVelocityPolytope";

    polytope_volumes.volumes.resize ( 4 );

    while ( ros::ok() ) {
    	
        if ( joint_state_received==true ) {
            joint_state_received=false;

            if(lin_callback_received==true)
            {
             lin_callback_received=false;
             robot_polytope.setLinearizationLimit(lin_limit.data);
            }
            
            if(!robot_polytope.checkCollision ( joint_state ))
            {
                polytope_volumes.volumes[0]=  robot_polytope.getAllowableMotionPolytope ( joint_state,
                                              AHrep,
                                              bhrep,
                                              Vset,
                                              offset_position,
                                              show_mp,
                {0.0,0.0,0.5,0.0},
                {0.0,0.0,1.0,0.4});
                
                polytope_volumes.volumes[1]=  robot_polytope.getConstrainedAllowableMotionPolytope ( joint_state,   
                                                                                                     AHrep,
                                              bhrep,
                                              Vset,
                                              offset_position,
                                              show_cmp,
                {0.0,0.0,0.5,0.0},
                {1.0,0.0,0.0,0.4} );
                
                double plane_width=0.004; // it seems in rviz anyway if you go lower than this there are display issues
                // If this fdoesn't happen in unity you can reduce this 0.001 -> 1mm
                robot_polytope.slicePolytope(Vset, offset_position,
                            {0.0,0.0,0.5,0.0},
                            {0.0,0.0,0.8,1.0},
                            "xy_slice",
                            ConstrainedManipulability::SLICING_PLANE::XY_PLANE,0.004);
                         ros::spinOnce();
                           
                robot_polytope.slicePolytope(Vset, offset_position,
                            {0.0,0.0,0.5,0.0},
                            {0.0,0.8,0.0,1.0},
                            "xz_slice",
                            ConstrainedManipulability::SLICING_PLANE::XZ_PLANE,0.004);
                        ros::spinOnce();

                robot_polytope.slicePolytope(Vset, offset_position,
                            {0.0,0.0,0.5,0.0},
                            {0.8,0.0,0.0,1.0},
                            "yz_slice",
                            ConstrainedManipulability::SLICING_PLANE::YZ_PLANE,0.004);
                        ros::spinOnce();

     	    
                vol_pub.publish ( polytope_volumes );
            }
        }

        ros::spinOnce();
        ros::Duration ( 0.001 ).sleep();
    }
    return 0;

}
