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


    // TEST FOR STATIC FUNCTIONS
    KDL::Tree my_tree_;
    KDL::Chain chain_;
    urdf::Model model_;
      std::string robot_desc_string;
    nh.param ( "robot_description", robot_desc_string, std::string() );
    model_.initParamWithNodeHandle ( "robot_description",nh ); 
    if ( !kdl_parser::treeFromString ( robot_desc_string, my_tree_ ) ) {
        ROS_ERROR ( "Failed to construct kdl tree" );
    } else {
        ROS_INFO ( "Success" );
    }
    my_tree_.getChain ( root,
                        tip,
                        chain_ );


    objects.resize ( shapes_in.size() );
    for ( int i = 0; i < shapes_in.size(); ++i ) {
        robot_polytope.addCollisionObject ( shapes_in[i],shapes_pose[i],i );
        objects[i].object_shape=shapes_in[i];
        objects[i].object_transform=shapes_pose[i];
    }

    robot_polytope.displayObjects();
    

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    bool show_staticresults(false);
    while ( ros::ok() ) {
        if ( recieved==true ) {



            recieved=false;
            ROS_INFO ( "------Checking Collision------" );
            std::cout<<" In Collision "<<robot_polytope.checkCollision ( joint_state ) <<std::endl;
            //std::cout<<robot_polytope.displayCollisionModel ( joint_state ) <<std::endl;
            ros::Duration ( 0.05 ).sleep();
	    
            ROS_INFO ( "------Get Allowable Motion Polytope volume------" );
            std::cout<<robot_polytope.getAllowableMotionPolytope ( joint_state,true ) <<std::endl;
            ros::Duration ( 0.05 ).sleep();
	    
	    
            ROS_INFO ( "------Get Constrained Allowable Motion Polytope volume------" );
            std::cout<<robot_polytope.getConstrainedAllowableMotionPolytope ( joint_state,true,{0.0,0.0,0.5,0.1},{1.0,0.0,0.0,0.1} ) <<std::endl;
            ros::Duration ( 0.05 ).sleep();
	    
            ROS_INFO ( "------Get velocity Motion Polytope volume------" );
            std::cout<<robot_polytope.getVelocityPolytope ( joint_state,false ) <<std::endl;
            ros::Duration ( 0.05 ).sleep();
	    
            ROS_INFO ( "------Get Constrained Velocity Motion Polytope volume-----" );
            std::cout<<robot_polytope.getConstrainedVelocityPolytope ( joint_state,false ) <<std::endl;	    
            ros::Duration ( 0.05 ).sleep();
	    
	    
	    
	    
	    std::cout<<"========================================================="<<std::endl;
	  
	    if(show_staticresults)
	    {
	    
            ROS_INFO ( "------Get Allowable Motion Polytope------" );
            std::cout<<ConstrainedManipulability::getAllowableMotionPolytope ( chain_,model_,joint_state,A,b ) <<std::endl;
            ros::Duration ( 0.05 ).sleep();
	    
	    
            ROS_INFO ( "------Get Constrained Allowable Motion Polytope------" );
            std::cout<<ConstrainedManipulability::getConstrainedAllowableMotionPolytope ( chain_,model_,objects,joint_state,A,b ) <<std::endl;
            ros::Duration ( 0.05 ).sleep();
	    
            ROS_INFO ( "------Get velocity Motion Polytope------" );
	    std::cout<<ConstrainedManipulability::getVelocityPolytope ( chain_,model_,joint_state,A,b ) <<std::endl;
            ros::Duration ( 0.05 ).sleep();
            ROS_INFO ( "------Get Constrained Velocity Motion Polytope------" );
	    std::cout<<ConstrainedManipulability::getConstrainedVelocityPolytope (chain_,model_,objects,joint_state,A,b) <<std::endl;
            ros::Duration ( 0.05 ).sleep();
	    
	    }


	    std::cout<<"========================================================="<<std::endl;
	                
        }
        ros::spinOnce();

    }


    return 0;

}
