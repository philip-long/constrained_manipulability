#include <constrained_manipulability/constrained_manipulability.h>
#include <random>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

// Initialisation of joint state relies on these
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_manager_msgs/SwitchController.h>

#include "snoptProblem.hpp"

sensor_msgs::JointState joint_state;
geometry_msgs::Twist teleop_twist;
bool screw_trigger ( false );
bool joint_state_received ( false );
bool twist_received ( false );
bool ignore_constraints( false );


std::vector<double> q_reference_g; // reference joint position
Eigen::MatrixXd AHrep_g; // Hyperplane constraints
Eigen::VectorXd bHrep_g,shift_to_sampled_joint_state (6); // Hyperplane constraints
Eigen::VectorXd desired_twist_g(6);
Eigen::Matrix<double,6,6> Jacobian_g; // Global Jacobian

void calculateDeviationAbs ( int    *Status, int *n,    double x[],
                             int    *needF,  int *neF,  double F[],
                             int    *needG,  int *neG,  double G[],
                             char      *cu,  int *lencu,
                             int    iu[],    int *leniu,
                             double ru[],    int *lenru );

void screwCallback ( const std_msgs::Bool::ConstPtr& msg ) {
    screw_trigger=msg->data;
}

void jointSensorCallback ( const sensor_msgs::JointState::ConstPtr& msg ) {
    joint_state=*msg;
    // Mod joint state wrist_3_joint to not exceed 2*PI
    joint_state.position[5] = fmod(joint_state.position[5], (2*M_PI));
    joint_state_received=true;
}

void twistCallback ( const geometry_msgs::Twist::ConstPtr& msg ) {
    teleop_twist=*msg;
    twist_received=true;
}

Eigen::VectorXd  convertGeometryTwistEigenVector(const geometry_msgs::Twist geo_twist_)
{
    Eigen::VectorXd desired_twist_(6);
    desired_twist_(0)=geo_twist_.linear.x;
    desired_twist_(1)=geo_twist_.linear.y;
    desired_twist_(2)=geo_twist_.linear.z;
    desired_twist_(3)=geo_twist_.angular.x;
    desired_twist_(4)=geo_twist_.angular.y;
    desired_twist_(5)=geo_twist_.angular.z;
    return desired_twist_;
}

trajectory_msgs::JointTrajectory jointStateToTraj(const sensor_msgs::JointState& joint, double time_from_start) {
    if (joint.name.empty() || joint.position.empty()) {
        ROS_ERROR("No joint names or positions specified");
    }

    trajectory_msgs::JointTrajectory traj;
    traj.header = joint.header;

    // Fill in joint names to trajectory
    for (std::vector<std::string>::const_iterator it = joint.name.begin(); it != joint.name.end(); ++it) {
        traj.joint_names.push_back(*it);
    }

    // Add a single point to the trajectory, expected arrival after 'time_from_start' seconds
    trajectory_msgs::JointTrajectoryPoint traj_point;
    for (std::vector<double>::const_iterator it = joint.position.begin(); it != joint.position.end(); ++it) {
        traj_point.positions.push_back(*it);
        traj_point.velocities.push_back(0);
        traj_point.accelerations.push_back(0);
    }
    traj_point.time_from_start = ros::Duration(time_from_start);
    traj.points.push_back(traj_point);

    return traj;
}

void sampleJointStates(sensor_msgs::JointState current_state,
                       std::vector<sensor_msgs::JointState>& sampled_state,
                       double deviation
                      )
{
    if (deviation == 0.0) {
        ROS_ERROR("No deviation value provided");
    }

    double min_deviation=-deviation;
    double max_deviation=deviation;
    bool initial(true);

    for (auto& one_sample_state:sampled_state) {
        one_sample_state=current_state;
        if(initial)
        {
            initial=false; // don't change first sample
        }
        else
        {
            for ( auto& one_joint_position:one_sample_state.position) {
                one_joint_position += (min_deviation + (double)(rand()) / ((double)(RAND_MAX/(max_deviation - min_deviation))) );
            }
        }
    }
}


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "ik_example" );
    std::srand(std::time(nullptr));

    ROS_INFO ( "UR3-e" );
    ros::NodeHandle nh; // Create a node handle and start the node

    constrained_manipulability::PolytopeVolume polytope_volumes;

    ros::Subscriber screw_sub = nh.subscribe ("/screw_trigger", 1, &screwCallback);
    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 1, &jointSensorCallback);
    ros::Subscriber vel_sub = nh.subscribe ("/cmd_vel", 1, &twistCallback);

    ros::Publisher vol_pub=nh.advertise<constrained_manipulability::PolytopeVolume>("constrained_manipulability/polytope_volumes", 1);

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1, true); // latched publishers
    ros::Publisher joint_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_traj", 10);
    ros::Publisher joint_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_command", 10);
    ros::Publisher end_effector_pub = nh.advertise<geometry_msgs::PoseStamped>("end_effector_pose", 10);

    ros::ServiceClient switch_client = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController switch_srv;
    
    std::string root,tip,pos_control,vel_control;
    std::vector<int> object_primitive;
    std::vector<std::vector<double>> obj_dimensions;
    std::vector<std::vector<double>> obj_poses;
    std::vector<shape_msgs::SolidPrimitive> shapes_in;
    TransformVector shapes_pose;
    FCLObjectSet objects;
    bool show_mp,show_cmp,op_constraints,debug_statements;
    int samples;
    double deviation;

    utility_functions::getParameter ( "~/ignore_constraints",op_constraints );
    utility_functions::getParameter ( "~/debug_statements",debug_statements );
    utility_functions::getParameter ( "~/root",root );
    utility_functions::getParameter ( "~/tip",tip );
    utility_functions::getParameter ( "~/pos_controller",pos_control );
    utility_functions::getParameter ( "~/vel_controller",vel_control );
    utility_functions::getParameter ( "~/show_mp",show_mp );
    utility_functions::getParameter ( "~/show_cmp",show_cmp );
    utility_functions::getParameter ( "~/num_samples",samples );
    utility_functions::getParameter ( "~/search_deviation",deviation );
    utility_functions::getVectorParam ( "~/object_primitive",object_primitive );
    utility_functions::getVectorVectorParam ( "~/object_dimensions",obj_dimensions );
    utility_functions::getVectorVectorParam ( "~/object_poses",obj_poses );
    utility_functions::getCollisionShapes ( object_primitive,
                                            obj_dimensions,
                                            obj_poses,
                                            shapes_in,
                                            shapes_pose );

    ignore_constraints=op_constraints;
    ROS_WARN_COND(ignore_constraints,"Ignoring Constraints");
    
    sensor_msgs::JointState pub_joint_state;
    pub_joint_state.name = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    pub_joint_state.position = {-0.080285145591739, -2.26159764473425, 2.32355683318005, -3.08731291385277, -1.65928451987101, 0.000698131700798};

    trajectory_msgs::JointTrajectory traj_state;
    std_msgs::Float64MultiArray joint_cmd;
    joint_cmd.data.resize(6);

    ConstrainedManipulability robot_polytope ( nh,root,tip );

    objects.resize ( shapes_in.size() );
    for ( int i = 0; i < shapes_in.size(); ++i ) {
        robot_polytope.addCollisionObject ( shapes_in[i],shapes_pose[i],i );
        objects[i].object_shape=shapes_in[i];
        objects[i].object_transform=shapes_pose[i];
    }
    ros::Duration ( 2.0 ).sleep();
    robot_polytope.displayObjects();

    polytope_volumes.names.resize ( 2 );
    polytope_volumes.names[0]="AllowableMotion";
    polytope_volumes.names[1]="ConstrainedAllowableMotion";
    polytope_volumes.volumes.resize ( 2 );

    std::vector<sensor_msgs::JointState> vec_sampled_joint_states(samples);

    Eigen::MatrixXd AHrep,Vset;
    Eigen::VectorXd bhrep;
    Eigen::Vector3d offset_position;
    Eigen::Matrix<double,6,Eigen::Dynamic> Jacobian;
    std::vector<double> joint_deviation ( 6 ); // Resulting change in joint position

    pub_joint_state.header.seq=0;
    pub_joint_state.header.seq++;
    pub_joint_state.header.stamp=ros::Time::now();
    joint_pub.publish(pub_joint_state);

    // Convert joint state message to a trajectory for command control
    traj_state = jointStateToTraj(pub_joint_state, 5);
    joint_traj_pub.publish(traj_state);

    ros::spinOnce();
    ros::Duration ( 5.0 ).sleep();

    switch_srv.request.start_controllers.push_back(vel_control);
    switch_srv.request.stop_controllers.push_back(pos_control);
    switch_srv.request.strictness = switch_srv.request.BEST_EFFORT;
    if (switch_client.call(switch_srv))
    {
        ROS_INFO("Successfully switched controllers!");
    }
    else
    {
        ROS_ERROR("Failed to call service switch_controller");
    }

    joint_state=pub_joint_state;

    twist_received=false;
    double objective_function=250.0;
    while ( ros::ok() ) {
        geometry_msgs::PoseStamped current;
        robot_polytope.getCartPos(joint_state, current.pose);
        current.header = joint_state.header;
        end_effector_pub.publish(current);

        if(!screw_trigger) 
        {
            // UNCOMMENT if in simulation
            //joint_state=pub_joint_state;

            // This function simply finds random joint configurations in the neighborhood
            sampleJointStates(joint_state, vec_sampled_joint_states, deviation);

            if(twist_received)
            {
                twist_received=false;
                double color_a=0.0;
                int sample_number=0;
                // Cycle through the sampled joints and display the polytope for each one
                // SNOPT stuff
                
                objective_function=100.0;
                for (auto& sample_joint_state:vec_sampled_joint_states)
                {
                    sample_number++;
                    color_a+=0.2;
                    // unconstrained polytope
                    double allowable_vol = robot_polytope.getAllowableMotionPolytope( sample_joint_state,
                                            AHrep,
                                            bhrep,
                                            Vset,
                                            offset_position,
                                            show_mp,
                                            {0.0,0.0,0.5,0.0},
                                            {0.0,0.0,1.0,0.4});

                    // constrained polytope
                    double allowable_vol_constrained = robot_polytope.getConstrainedAllowableMotionPolytope( sample_joint_state,
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
                                ConstrainedManipulability::SLICING_PLANE::XY_PLANE,plane_width);
                             ros::spinOnce();
                               
                    robot_polytope.slicePolytope(Vset, offset_position,
                                {0.0,0.0,0.5,0.0},
                                {0.0,0.8,0.0,1.0},
                                "xz_slice",
                                ConstrainedManipulability::SLICING_PLANE::XZ_PLANE,plane_width);
                            ros::spinOnce();

                    robot_polytope.slicePolytope(Vset, offset_position,
                                {0.0,0.0,0.5,0.0},
                                {0.8,0.0,0.0,1.0},
                                "yz_slice",
                                ConstrainedManipulability::SLICING_PLANE::YZ_PLANE,plane_width);
                            ros::spinOnce();

                    robot_polytope.getJacobian(sample_joint_state,Jacobian);

                    ROS_INFO_COND(debug_statements,"allowable_vol %f",allowable_vol);
                    ROS_INFO_COND(debug_statements,"allowable_vol_constrained %f",allowable_vol_constrained);
                    polytope_volumes.volumes[0] = allowable_vol;
                    polytope_volumes.volumes[1] = allowable_vol_constrained;

                    if(!robot_polytope.checkCollision(sample_joint_state))
                    {
                        
                        ROS_INFO_COND(debug_statements,"\n ===Starting SNOPT OPTIMZATION for %d ==== \n",sample_number);
                        
                        
                        Eigen::VectorXd dq ( Jacobian_g.cols() );
                        dq.setZero();
                        Eigen::VectorXd poly_constraints;
                        Eigen::VectorXd v ( Jacobian_g.rows() );


                        // Distance from where robot currently is to sampled state
                        for ( int j = 0; j < Jacobian_g.cols() ; ++j ) {
                            shift_to_sampled_joint_state ( j ) = sample_joint_state.position[j] - joint_state.position[j];
                        }

                        // Assign Global Variables
                        AHrep_g=AHrep;
                        bHrep_g=bhrep;
                        Jacobian_g=Jacobian;
                        desired_twist_g=convertGeometryTwistEigenVector(teleop_twist);


                        snoptProblemA onlineOptimization;
                        onlineOptimization.initialize ( "", (int) debug_statements); // no print file; summary on
                        onlineOptimization.setProbName ( "Online" );
                        onlineOptimization.setIntParameter ( "Derivative option", 0 ); // snopta will compute the Jacobian by finite-differences
                        onlineOptimization.setIntParameter ( "Verify level ", 0 );


                        int n     =  Jacobian_g.cols(); // decis vars joint velocities
                        int neF   =  1 + bHrep_g.rows(); // either cost + number of hyperplanes
                        int nS = 0, nInf;
                        double sInf;
                        double *x      = new double[n]; // decision variables
                        double *xlow   = new double[n]; // lb state, torque
                        double *xupp   = new double[n]; // up
                        double *xmul   = new double[n];
                        int    *xstate = new    int[n];

                        double *F      = new double[neF]; // contain cost and constraints
                        double *Flow   = new double[neF]; // F lower bound, non linear constraints
                        double *Fupp   = new double[neF];
                        double *Fmul   = new double[neF];
                        int    *Fstate = new int[neF];

                        Flow[0] =  9e-06;
                        Fupp[0] =  1e20;
                        Fmul[0] =   0;
                        int    ObjRow  = 0; // which elemnt in F is obj function
                        double ObjAdd  = 0; // don't touch

                        int Cold = 0, Basis = 1, Warm = 2; // cold start

                        // Philip Filling out variables
                        for ( int var = 0; var < n; ++var ) {
                            x[var] = 0.0; // initialize at current torque values
                            xlow[var] = -1e20;
                            xupp[var] = 1e20;
                            xstate[var] = 0;
                        }

                        double tolerance = 0.001;

                        for ( int var = 1; var < neF; ++var ) {
                            Flow[var] = -1e20;
                            Fupp[var] = bhrep ( var - 1 ) - tolerance;
                        }

                        onlineOptimization.solve ( Cold, neF, n, ObjAdd, ObjRow, calculateDeviationAbs,
                                                   xlow, xupp, Flow, Fupp,
                                                   x, xstate, xmul, F, Fstate, Fmul,
                                                   nS, nInf, sInf );

                        joint_deviation.assign ( x, x+n ); // pointer magic
                        if(debug_statements) utility_functions::printVector(joint_deviation,"resulting joint deviation");

                        Eigen::VectorXd dq_sol=utility_functions::vectorToEigen(joint_deviation);

                        
                        if(*F<objective_function)
                        {
                            objective_function=*F;

                            ROS_INFO_COND(debug_statements,"Updating value based on %d",sample_number);
                            
                            Eigen::VectorXd twist_shifted_output = Jacobian_g * (dq_sol + shift_to_sampled_joint_state);
                            if(debug_statements) std::cout<<"twist_shifted_output  "<<twist_shifted_output<<std::endl;
                            if(debug_statements) std::cout<<"desired_twist_g value "<<desired_twist_g<<std::endl;
                            
                            Eigen::VectorXd vel_err= ( twist_shifted_output - desired_twist_g );
                            double sum_squared_error=vel_err.dot ( vel_err );
                            ROS_INFO_COND(debug_statements,"sum_squared_error %f",sum_squared_error);
                            for ( int j = 0; j < Jacobian_g.cols() ; ++j ) {
                                pub_joint_state.position[j] = dq_sol( j ) + sample_joint_state.position[j];
                                // Equivalent to dq_star ( j )
                                // Creating a velocity command out of the shifted IK solution
                                joint_cmd.data[j] = dq_sol( j ) + shift_to_sampled_joint_state ( j );
                            }
                        }
                    }
                    else
                    {
                        ROS_WARN_COND(debug_statements,"in collision optimization will fail");
                    }
                }

                // Publish computed volumes
                vol_pub.publish(polytope_volumes);

                // Position trajectory control
                //traj_state = jointStateToTraj(pub_joint_state, 0.5);
                //joint_traj_pub.publish(traj_state);
            }
        }
        else
        {
            // Rotate the screw at 0.3 rad/s
            joint_cmd.data[5] = 0.3;
        }


        // UNCOMMENT if in simulation
        //pub_joint_state.header.seq++;
        //pub_joint_state.header.stamp=ros::Time::now();
        //joint_pub.publish(pub_joint_state);

        // Velocity control
        joint_cmd_pub.publish(joint_cmd);
        // Reset to 0 after publishing velocity command
        std::fill(joint_cmd.data.begin(), joint_cmd.data.end(), 0.0);

        ros::spinOnce();
        ros::Duration ( 0.001 ).sleep();
    }

    return 0;
}



// In this case decision variables is deviation from reference position
void calculateDeviationAbs ( int    *Status, int *n,    double x[],
                             int    *needF,  int *neF,  double F[],
                             int    *needG,  int *neG,  double G[],
                             char      *cu,  int *lencu,
                             int    iu[],    int *leniu,
                             double ru[],    int *lenru ) {
    Eigen::VectorXd dq ( Jacobian_g.cols() );
    Eigen::VectorXd dq_star ( Jacobian_g.cols() );


    Eigen::VectorXd poly_constraints;
    Eigen::VectorXd v ( Jacobian_g.rows() );

    poly_constraints.resize ( AHrep_g.rows() );
    poly_constraints.setZero();
    std::vector<double> q_new ( Jacobian_g.cols() );

    // Robot is at a position  corresponds to current_joint_state
    // We have found a polytope in neighborhood at q_sample
    // This distance  is joint_shift_to_sampled_poly_origin=sample_joint_state-current_joint_state
    // We want to know that if you move dq to satisfy twist will that movement still be within polytope
    //



    for ( int i = 0; i < Jacobian_g.cols(); ++i ) {
        dq ( i ) = x[i];
        dq_star ( i ) = dq ( i ) + shift_to_sampled_joint_state ( i );
    }



    // Desired input is from current position we need to shift it to sampled origin
    // We can't do this in the constraints as constants are ignored
    v=Jacobian_g*dq_star; //
    Eigen::VectorXd vel_err= ( v- desired_twist_g);
    double sum_squared_error=100000*(vel_err.dot ( vel_err ));

    if(ignore_constraints)
    {
        for ( int i = 0; i < AHrep_g.rows(); ++i ) {
            F[i + 1] = 0.0;
        }
    }
    else
    {
        poly_constraints = ( AHrep_g*dq); // tolerance
        for ( int i = 0; i < AHrep_g.rows(); ++i ) {
            poly_constraints ( i );
            F[i + 1] = poly_constraints ( i );
        }
    }
    F[0] = sum_squared_error;
}
