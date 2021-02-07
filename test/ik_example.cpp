#include <constrained_manipulability/constrained_manipulability.h>
#include <random>
#include <geometry_msgs/Twist.h>
#include "snoptProblem.hpp"

sensor_msgs::JointState joint_state;
geometry_msgs::Twist teleop_twist;
bool joint_state_received ( false );
bool twist_received ( false );

std::vector<double> q_reference_g; // reference joint position
Eigen::MatrixXd AHrep_g; // Hyperplane constraints
Eigen::VectorXd bHrep_g,dq_current (6); // Hyperplane constraints
Eigen::VectorXd desired_twist_g(6);
Eigen::Matrix<double,6,6> Jacobian_g; // Global Jacobian

void calculateDeviationAbs ( int    *Status, int *n,    double x[],
                             int    *needF,  int *neF,  double F[],
                             int    *needG,  int *neG,  double G[],
                             char      *cu,  int *lencu,
                             int    iu[],    int *leniu,
                             double ru[],    int *lenru );


void jointSensorCallback ( const sensor_msgs::JointState::ConstPtr& msg ) {
    joint_state=*msg;
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

void sampleJointStates(sensor_msgs::JointState current_state, 
                       std::vector<sensor_msgs::JointState>& sampled_state
                      )
{
    double min_deviation=-0.3;
    double max_deviation=0.3;
    bool initial(true);
    
    for ( auto& one_sample_state:sampled_state) {
        one_sample_state=current_state;
        if(initial)
        {
            initial=false; // don't change first sample
        }
        else
        {
            for ( auto& one_joint_position:one_sample_state.position) {
                one_joint_position+= (min_deviation + (double)(rand()) / ((double)(RAND_MAX/(max_deviation - min_deviation))) );
            }
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
    ros::Subscriber  vel_sub= nh.subscribe ( "/cmd_vel",
                              1, &twistCallback );

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

    std::vector<sensor_msgs::JointState> sampled_joint_state(3);

    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
    Eigen::Matrix<double,6,Eigen::Dynamic> Jacobian;
    std::vector<double> joint_deviation ( 7 ); // Resulting change in joint position


    while ( ros::ok() ) {

        if ( joint_state_received==true ) {
            joint_state_received=false;

            // This function simply finds random joint configurations in the neighborhood
            sampleJointStates(joint_state,sampled_joint_state);




            if(twist_received)
            {
                twist_received=false;
                double color_a=0.0;
                int sample_number=0;
                // Cycle through the sampled joints and display the polytope for each one
                // SNOPT stuff
                for ( auto& one_sample:sampled_joint_state)
                {
                    sample_number++;
                    color_a+=0.2;
                    // unconstrained polytope
                    double allowable_vol=robot_polytope.getAllowableMotionPolytope( one_sample,
                                         show_mp,
                    {0.0,0.0,0.5,0.0},
                    {0.0,0.0,color_a,0.4} );

                    // constrained polytope
                    double allowable_vol_constrained=robot_polytope.getConstrainedAllowableMotionPolytope( one_sample,
                                                     AHrep,
                                                     bhrep,
                                                     show_mp,
                    {0.0,0.0,0.5,0.0},
                    {color_a,0.0,0.0,0.4} );

                    robot_polytope.getJacobian(one_sample,Jacobian);

                    std::cout<<"allowable_vol"<<allowable_vol_constrained<<std::endl;
                    std::cout<<"allowable_vol_constrained"<<allowable_vol_constrained<<std::endl;

                    
            
                    if(!robot_polytope.checkCollision(one_sample))
                    {
                        std::cout<<"\n =================================== \n"<<std::endl;
                        std::cout<<"\n ===Starting SNOPT OPTIMZATION "<<sample_number<<"===== \n"<<std::endl;
                        std::cout<<"\n =================================== \n"<<std::endl;
                        Eigen::VectorXd dq ( Jacobian_g.cols() );
                        dq.setZero();
                        Eigen::VectorXd poly_constraints;
                        Eigen::VectorXd v ( Jacobian_g.rows() );

                        
                        // Distance from where robot currently is to sampled state
                        for ( int j = 0; j < Jacobian_g.cols() ; ++j ) {
                            dq_current ( j ) = one_sample.position[j] - joint_state.position[j]; 
                        }
                        std::cout<<"dq_current"<<dq_current<<std::endl;

                        ros::Duration ( 1.0 ).sleep();
                        

                        // Assign Global Variables
                        AHrep_g=AHrep;
                        bHrep_g=bhrep;
                        Jacobian_g=Jacobian;
                        desired_twist_g=convertGeometryTwistEigenVector(teleop_twist);


                        snoptProblemA onlineOptimization;
                        onlineOptimization.initialize ( "",1 ); // no print file; summary on
                        onlineOptimization.setProbName ( "Online" );
                        onlineOptimization.setIntParameter ( "Derivative option", 1 ); // snopta will compute the Jacobian by finite-differences
                        onlineOptimization.setIntParameter ( "Verify level ", 0 );
                        onlineOptimization.setRealParameter ( "Function Precision", 1e-10 );
                        onlineOptimization.setRealParameter ( "Major feasibility tolerance",  1.0e-4 ); // target nonlinear constraint violation
                        onlineOptimization.setRealParameter ( "Minor feasibility tolerance",  1.0e-4 ); // for satisfying the QP bounds
                        onlineOptimization.setRealParameter ( "Major optimality tolerance",   1.0e-3 ); // target complementarity gap

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

                        joint_deviation.assign ( x,x+n );
                        utility_functions::printVector(joint_deviation,"resulting joint deviation");
                        Eigen::VectorXd dq_sol=utility_functions::vectorToEigen(joint_deviation);
                        Eigen::VectorXd twist_output=Jacobian_g*dq_sol;
                        std::cout<<"Input twist"<<desired_twist_g<<std::endl;
                        std::cout<<"Output twist"<<desired_twist_g<<std::endl;

                    }
                    else
                    {
                        std::cout<<"in collision optimzation will fail"<<std::endl;
                    }
                }


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



// In this case decision variables is deviation from reference position
void calculateDeviationAbs ( int    *Status, int *n,    double x[],
                             int    *needF,  int *neF,  double F[],
                             int    *needG,  int *neG,  double G[],
                             char      *cu,  int *lencu,
                             int    iu[],    int *leniu,
                             double ru[],    int *lenru ) {
    Eigen::VectorXd dq ( Jacobian_g.cols() );
    Eigen::VectorXd dq_local ( Jacobian_g.cols()  );
    
    
    Eigen::VectorXd poly_constraints;
    Eigen::VectorXd v ( Jacobian_g.rows() );

    poly_constraints.resize ( AHrep_g.rows() );
    poly_constraints.setZero();
    std::vector<double> q_new ( Jacobian_g.cols() );

    // Robot is at a position  corresponds to current_joint_state 
    // We have found a polytope in neighborhood at q_sample    
    // This distance way is dq_current=one_sample-current_joint_state 
    // We want to know that if you move dq to satisfy twist will that movement still be within polytope
    //
    //Two examples to understand
    //  1. Let's say movement to realise twist exactly corresponds to dq_current
    //  Then we should have zero distance from one_sample's polytope origin dq_local=dq-dq_current=0 
    //  So we check dq_local against polytope constraints, A * dq_local <b, and it's zero since we are at origin
    //
    //  2. Let's say movement to realise twist is zero
    //  dq_local=dq-dq_current=-dq_current 
    // In this case it's as if robot was at polytope origin and is moving in dq_current so we check if that's in polytop
    
    // still something wrong here
    
    for ( int i = 0; i < Jacobian_g.cols(); ++i ) {
        dq ( i ) = x[i];
        dq_local ( i ) =dq ( i ) - dq_current ( i ); 
    }
    
    
    
    v=Jacobian_g*dq; // need to think about this
    Eigen::VectorXd vel_err= ( v- desired_twist_g);
    double sum_squared_error=vel_err.dot ( vel_err );

    poly_constraints = ( AHrep_g * dq_local ); // tolerance

    for ( int i = 0; i < AHrep_g.rows(); ++i ) {
        poly_constraints ( i );
        F[i + 1] = poly_constraints ( i );
    }
    F[0] = 1000*sum_squared_error;
}
