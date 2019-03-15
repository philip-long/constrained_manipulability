#include <constrained_manipulability/constrained_manipulability.h>


sensor_msgs::JointState joint_state;
bool recieved ( false );

void jointSensorCallback ( const sensor_msgs::JointState::ConstPtr& msg ) {
    joint_state=*msg;
    recieved=true;
}

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "fcl_sawyer" ); // ros init
    ROS_INFO ( "FCL SAWYER" );
    ros::NodeHandle nh; // Create a node handle and start the node

    ros::Subscriber  joint_sub= nh.subscribe ( "/joint_states",
                                1, &jointSensorCallback );

    std::string root,tip;
    utility_functions::getParameter ( "~/root",root );
    utility_functions::getParameter ( "~/tip",tip );

    ConstrainedManipulability robot_polytope ( nh,root,tip );

    ros::Duration ( 1.0 ).sleep();

    std::vector<int> object_primitive;
    std::vector<std::vector<double>> obj_dimensions;
    std::vector<std::vector<double>> obj_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes_in;
    TransformVector shapes_pose;
    FCLObjectSet objects;


    utility_functions::getVectorParam ( "~/object_primitive",object_primitive );
    utility_functions::getVectorVectorParam ( "~/object_dimensions",obj_dimensions );
    utility_functions::getVectorVectorParam ( "~/object_poses",obj_poses );
    utility_functions::getCollisionShapes ( object_primitive,
                                            obj_dimensions,
                                            obj_poses,
                                            shapes_in,
                                            shapes_pose );


    for ( int i = 0; i < shapes_in.size(); ++i ) {
        robot_polytope.addCollisionObject ( shapes_in[i],shapes_pose[i],i );
    }

    robot_polytope.displayObjects();

    while ( ros::ok() ) {
        if ( recieved==true ) {



            recieved=false;
            ROS_INFO ( "------Displaying Robot Collision Model------" );
            //std::cout<<" In Collision "<<robot_polytope.checkCollision ( joint_state ) <<std::endl;
            std::cout<<robot_polytope.displayCollisionModel ( joint_state,true ) <<std::endl;
//             ros::Duration ( 0.1 ).sleep();
// 	    ROS_INFO ( "------Get Allowable Motion Polytope------" );
//             std::cout<<robot_polytope.getAllowableMotionPolytope ( joint_state,true )<<std::endl;
//             ros::Duration ( 0.1 ).sleep();
// 	    ROS_INFO ( "------Get Constrained Allowable Motion Polytope------" );
//             std::cout<<robot_polytope.getConstrainedAllowableMotionPolytope ( joint_state,true )<<std::endl;
//             ros::Duration ( 0.1 ).sleep();
// 	    ROS_INFO ( "------Get velocity Motion Polytope------" );
//             std::cout<<robot_polytope.getVelocityPolytope ( joint_state,false )<<std::endl;
//             ros::Duration ( 0.1 ).sleep();
// 	    ROS_INFO ( "------Get Constrained Motion Polytope------" );
//             std::cout<<robot_polytope.getConstrainedVelocityPolytope ( joint_state,false )<<std::endl;
//             ros::Duration ( 0.1 ).sleep();




            ros::Duration ( 0.1 ).sleep();
        }
        ros::spinOnce();

    }


    return 0;

}
