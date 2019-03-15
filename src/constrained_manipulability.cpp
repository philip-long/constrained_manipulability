/*** INCLUDE FILES ***/
#include <constrained_manipulability/constrained_manipulability.h>


ConstrainedManipulability::ConstrainedManipulability ( ros::NodeHandle nh,
        std::string root,
        std::string tip,
        double distance_threshold,
        double linearization_limit,
        double dangerfield
                                                     ) :
nh_ ( nh ),rvizDisplay ( nh ), fclInterface ( nh ),distance_threshold_ ( distance_threshold ),dangerfield_ ( dangerfield ) {


    std::string robot_desc_string;
    nh_.param ( "robot_description", robot_desc_string, std::string() );
    model_.initParamWithNodeHandle ( "robot_description",nh ); //Maybe you can use other init~ function.
    if ( !kdl_parser::treeFromString ( robot_desc_string, my_tree_ ) ) {
        ROS_ERROR ( "Failed to construct kdl tree" );
    } else {
        ROS_INFO ( "Success" );
    }


    base_link_=root;
    my_tree_.getChain ( root,
                        tip,
                        chain_ );

    ndof_=chain_.getNrOfJoints();

    ROS_INFO_STREAM ( "ndof_= "<<ndof_ );
    std::cout<<"ndof_= "<<ndof_<<std::endl;
    std::cout<<"Number of tree segments_= "<<my_tree_.getNrOfSegments() <<std::endl;
    std::cout<<"Number of Joints= "<<my_tree_.getNrOfJoints() <<std::endl;
    std::cout<<"Number of chain segments_= "<<chain_.getNrOfSegments() <<std::endl;
    std::cout<<"Number of chain joints= "<<chain_.getNrOfJoints() <<std::endl;

    qmax_.resize ( ndof_ );
    qmin_.resize ( ndof_ );
    qdotmax_.resize ( ndof_ );
    qdotmin_.resize ( ndof_ );
    max_lin_limit_.resize ( ndof_ );
    min_lin_limit_.resize ( ndof_ );
    // Default values

    urdf::LinkConstSharedPtr link;
    urdf::JointSharedPtr j;

    KDL::JntArray kdl_joint_positions = KDL::JntArray ( ndof_ );

    for ( unsigned int i=0; i<ndof_; i++ ) {
        kdl_joint_positions ( i ) =0.1;
    }

    kdl_fk_solver_.reset ( new KDL::ChainFkSolverPos_recursive ( chain_ ) );
    kdl_dfk_solver_.reset ( new KDL::ChainJntToJacSolver ( chain_ ) );



    int mvable_jnt ( 0 );
    for ( int i=0; i<ndof_+1; ++i ) {
        KDL::Segment seg=chain_.getSegment ( i );
        KDL::Joint kdl_joint=seg.getJoint();

        if ( kdl_joint.getType() !=KDL::Joint::None ) {
            qmax_[mvable_jnt]=model_.joints_.at ( kdl_joint.getName() )->limits->upper;
            qmin_[mvable_jnt]=model_.joints_.at ( kdl_joint.getName() )->limits->lower;

            qdotmax_[mvable_jnt]=model_.joints_.at ( kdl_joint.getName() )->limits->velocity;
            qdotmin_[mvable_jnt]=-model_.joints_.at ( kdl_joint.getName() )->limits->velocity;


            mvable_jnt++;
        }

    }

    ndof_identity_matrix_.resize ( ndof_,ndof_ );
    ndof_identity_matrix_.setZero();
    for ( int i = 0; i<ndof_identity_matrix_.rows(); i++ ) {
        for ( int j = 0; j<ndof_identity_matrix_.cols(); j++ ) {
            if ( i==j ) {
                ndof_identity_matrix_ ( i,j ) =1;
            }
        }
    }

    utility_functions::printVector ( qmax_,"qmax_" );
    utility_functions::printVector ( qmin_,"qmin_" );

    std::fill ( max_lin_limit_.begin(), max_lin_limit_.end(),linearization_limit );
    std::fill ( min_lin_limit_.begin(), min_lin_limit_.end(),-linearization_limit );

}



bool ConstrainedManipulability::addCollisionObject ( const shape_msgs::SolidPrimitive & s1,
        const  Eigen::Affine3d  & wT1,unsigned int object_id ) {
    fclInterface.addCollisionObject ( s1,wT1,object_id );
}

bool ConstrainedManipulability::addCollisionObject ( const shape_msgs::Mesh & s1,
        const  Eigen::Affine3d  & wT1,unsigned int object_id ) {
    fclInterface.addCollisionObject ( s1,wT1,object_id );
}

bool ConstrainedManipulability::addCollisionObject ( FCLObjectSet objects ) {
    fclInterface.addCollisionObject ( objects );
}

bool ConstrainedManipulability::displayObjects() {
    fclInterface.displayObjects ( base_link_ );
}

bool ConstrainedManipulability::checkCollision ( const sensor_msgs::JointState & joint_states ) {
    KDL::JntArray 	kdl_joint_positions ( ndof_ );
    jointStatetoKDLJointArray ( joint_states,kdl_joint_positions );
    std::vector<shape_msgs::SolidPrimitive> 	geometry_mkrs;
//     // Collision Link transforms
    TransformVector geometry_transforms;

    getCollisionModel ( kdl_joint_positions,geometry_mkrs,geometry_transforms );



    for ( int i = 0; i<geometry_mkrs.size(); i++ ) {
        if ( fclInterface.checkCollisionObjectWorld ( geometry_mkrs[i],
                geometry_transforms[i] )
           ) {
            return true;
        }
    }
    return false;
}



bool ConstrainedManipulability::checkCollision ( const sensor_msgs::JointState & joint_states,bool mesh ) {
    KDL::JntArray 	kdl_joint_positions ( ndof_ );
    jointStatetoKDLJointArray ( joint_states,kdl_joint_positions );
    GeometryInformation geometry_information;
    getCollisionModel ( kdl_joint_positions,geometry_information );



    for ( int i = 0; i<geometry_information.geometry_transforms.size(); i++ ) {
        shapes::ShapeMsg sj;
        shapes::constructMsgFromShape ( geometry_information.shapes[i].get(),sj );


        if ( sj.which() ==0 ) {
            if ( fclInterface.checkCollisionObjectWorld ( boost::get<shape_msgs::SolidPrimitive> ( sj ),
                    geometry_information.geometry_transforms[i] )
               ) {
                return true;
            } else if ( sj.which() ==1 ) {
                if ( fclInterface.checkCollisionObjectWorld ( boost::get<shape_msgs::Mesh> ( sj ),
                        geometry_information.geometry_transforms[i] )
                   ) {
                    return true;
                }

            }
        }
    }
    return false;
}




double ConstrainedManipulability::getPolytopeVolume ( Eigen::MatrixXd  vertices ) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected ( new pcl::PointCloud<pcl::PointXYZ> );
    double vol ( 0.0 );
    for ( int var = 0; var < vertices.rows(); ++var ) {
        pcl::PointXYZ p ( vertices ( var,0 ),
                          vertices ( var,1 ) ,
                          vertices ( var,2 ) );
        cloud_projected->points.push_back ( p );

    }
    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector< pcl::Vertices > polygons;
    try {
        chull.setComputeAreaVolume ( true );
        chull.setInputCloud ( cloud_projected );
        chull.reconstruct ( *cloud_hull,polygons );
    } catch ( ... ) {
        ROS_ERROR ( "qhull error" );
        return 0.0;
    }
    vol=chull.getTotalVolume();
    return vol;
}




double ConstrainedManipulability::getAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
        bool show_polytope,
        std::vector<double>  color_pts,
        std::vector<double>  color_line ) {
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;

    double vol=getAllowableMotionPolytope ( joint_states,
                                            AHrep,
                                            bhrep,
                                            show_polytope, color_pts,color_line );
    return vol;
}

double ConstrainedManipulability::getVelocityPolytope ( const sensor_msgs::JointState & joint_states,
        bool show_polytope,
        std::vector<double>  color_pts,
        std::vector<double>  color_line ) {
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;

    double vol=getVelocityPolytope ( joint_states,
                                     show_polytope,
                                     AHrep,
                                     bhrep,
                                     color_pts,color_line );
    return vol;
}


double ConstrainedManipulability::getVelocityPolytope ( const sensor_msgs::JointState & joint_states,
        bool show_polytope,
        Eigen::MatrixXd AHrep,
        Eigen::VectorXd bhrep,
        std::vector<double>  color_pts,
        std::vector<double>  color_line ) {


    double vol_initial ( -1 );
    KDL::JntArray 	kdl_joint_positions ( ndof_ );
    jointStatetoKDLJointArray ( joint_states,kdl_joint_positions );
    Eigen::Affine3d base_T_ee;
    KDL::Frame cartpos;
    KDL::Jacobian base_J_link_origin;
    base_J_link_origin.resize ( ndof_ );
    kdl_fk_solver_->JntToCart ( kdl_joint_positions,cartpos );
    tf::transformKDLToEigen ( cartpos,base_T_ee );
    kdl_dfk_solver_->JntToJac ( kdl_joint_positions,base_J_link_origin );
    Eigen::MatrixXd base_J_ee=base_J_link_origin.data;

    Eigen::MatrixXd Ahrep_undeformed;
    Eigen::VectorXd bhrep_underformed;
    Eigen::MatrixXd Qset_undeformed,Vset_undeformed;
    Ahrep_undeformed.resize ( 2*ndof_,ndof_ );
    Ahrep_undeformed.topRows ( ndof_ ) =ndof_identity_matrix_;  // ndof_*ndof block at row  0 colum 0;ndof_
    Ahrep_undeformed.block ( ndof_,0,ndof_,ndof_ ) =-ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_
    bhrep_underformed.resize ( 2*ndof_,1 );
    for ( int i = 0; i < ndof_; ++i ) {
        bhrep_underformed ( i ) =qdotmax_[i];
        bhrep_underformed ( i+ndof_ ) =-qdotmin_[i];
    }
    getVrepPolytope ( Ahrep_undeformed,bhrep_underformed,Qset_undeformed );
    getCartesianPolytope ( Qset_undeformed,base_J_ee.topRows ( 3 ),base_T_ee.translation(),Vset_undeformed );
    vol_initial=getPolytopeVolume ( Vset_undeformed );

    if ( show_polytope ) {
        rvizDisplay.plotPolytope ( "polytope", Vset_undeformed,base_T_ee.translation(), {0.0,0.0,0.5,1.0}, {0.0,0.0,1.0,0.8} );
    }
    return vol_initial;

}



double ConstrainedManipulability::getAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        bool show_polytope,
        std::vector<double>  color_pts,
        std::vector<double>  color_line ) {
    double vol_initial ( -1 );
    KDL::JntArray kdl_joint_positions ( ndof_ );
    std::cout<<"Joint states"<<std::endl;
    jointStatetoKDLJointArray ( joint_states,kdl_joint_positions );
    Eigen::Affine3d base_T_ee;
    KDL::Frame cartpos;
    KDL::Jacobian base_J_link_origin;
    base_J_link_origin.resize ( ndof_ );
    kdl_fk_solver_->JntToCart ( kdl_joint_positions,cartpos );
    tf::transformKDLToEigen ( cartpos,base_T_ee );
    kdl_dfk_solver_->JntToJac ( kdl_joint_positions,base_J_link_origin );
    Eigen::MatrixXd base_J_ee=base_J_link_origin.data;

    Eigen::MatrixXd Qset_undeformed,Vset_undeformed;


    AHrep.resize ( 2*ndof_,ndof_ );
    AHrep.topRows ( ndof_ ) =ndof_identity_matrix_;  // ndof_*ndof block at row  0 colum 0;ndof_
    AHrep.block ( ndof_,0,ndof_,ndof_ ) =-ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

    bhrep.resize ( 2*ndof_,1 );


    for ( int i = 0; i < ndof_; ++i ) {
        bhrep ( i ) =max_lin_limit_[i];
        bhrep ( i+ndof_ ) =-min_lin_limit_[i];
    }

    getVrepPolytope ( AHrep,bhrep,Qset_undeformed );
    getCartesianPolytope ( Qset_undeformed,base_J_ee.topRows ( 3 ),base_T_ee.translation(),Vset_undeformed );
    vol_initial=getPolytopeVolume ( Vset_undeformed );
    if ( show_polytope ) {
        rvizDisplay.plotPolytope ( "polytope", Vset_undeformed,base_T_ee.translation(), {0.0,0.0,0.5,1.0}, {0.0,0.0,1.0,0.8} );
    }
    return vol_initial;

}







double ConstrainedManipulability::getConstrainedAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
        bool show_polytope,
        std::vector<double>  color_pts,
        std::vector<double>  color_line ) {

    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
    double vol_initial ( -1 );
    double vol=getConstrainedAllowableMotionPolytope ( joint_states,
               AHrep,
               bhrep,
               show_polytope,
               color_pts,
               color_line ) ;

    return vol;
}




double ConstrainedManipulability::getConstrainedAllowableMotionPolytope ( const sensor_msgs::JointState & joint_states,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        bool show_polytope,
        std::vector<double>  color_pts,
        std::vector<double>  color_line ) {




    Eigen::MatrixXd Qset,Vset;

    double vol_reduced ( 0.0 );
    KDL::JntArray 	kdl_joint_positions ( ndof_ );
    jointStatetoKDLJointArray ( joint_states,kdl_joint_positions );


//     // Collision model
    std::vector<shape_msgs::SolidPrimitive> 	geometry_mkrs;
//     // Collision Link transforms
    TransformVector geometry_transforms;
//     // Collision Link Jacobians
    JacobianVector geometry_jacobians;


    getCollisionModel ( kdl_joint_positions,geometry_mkrs,geometry_transforms,geometry_jacobians );


    bool collision_free=getPolytopeHyperPlanes ( kdl_joint_positions,
                        geometry_mkrs,
                        geometry_transforms,
                        geometry_jacobians ,
                        AHrep,
                        bhrep );

    if ( !collision_free ) {
        return 0.0;
    }


    bool valid_poly=getVrepPolytope ( AHrep,bhrep,Qset );

    if ( !valid_poly ) {
        return 0.0;
    }
    getCartesianPolytope ( Qset,geometry_jacobians.back().topRows ( 3 ),geometry_transforms.back().translation(),Vset );
    if ( show_polytope ) {
        rvizDisplay.plotPolytope ( "polytope", Vset,geometry_transforms.back().translation(),color_pts, color_line );
    }

    vol_reduced=getPolytopeVolume ( Vset );
    return vol_reduced;
}

double ConstrainedManipulability::getConstrainedVelocityPolytope ( const sensor_msgs::JointState & joint_states,
        bool show_polytope,
        std::vector<double>  color_pts,
        std::vector<double>  color_line ) {
    Eigen::MatrixXd  AHrep;
    Eigen::VectorXd  bhrep;
    double vol=getConstrainedVelocityPolytope ( joint_states,
               AHrep,bhrep,
               show_polytope,
               color_pts,
               color_line ) ;
    return vol;
}

double ConstrainedManipulability::getConstrainedVelocityPolytope ( const sensor_msgs::JointState & joint_states,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        bool show_polytope,
        std::vector<double>  color_pts,
        std::vector<double>  color_line ) {




    Eigen::MatrixXd Qset,Vset;

    double vol_reduced ( 0.0 );
    KDL::JntArray 	kdl_joint_positions ( ndof_ );
    jointStatetoKDLJointArray ( joint_states,kdl_joint_positions );


//     // Collision model
    std::vector<shape_msgs::SolidPrimitive> 	geometry_mkrs;
//     // Collision Link transforms
    TransformVector geometry_transforms;
//     // Collision Link Jacobians
    JacobianVector geometry_jacobians;


    getCollisionModel ( kdl_joint_positions,geometry_mkrs,geometry_transforms,geometry_jacobians );


    bool collision_free=getPolytopeHyperPlanes ( kdl_joint_positions,
                        geometry_mkrs,
                        geometry_transforms,
                        geometry_jacobians ,
                        AHrep,
                        bhrep,
                        true );

    if ( !collision_free ) {
        return 0.0;
    }


    bool valid_poly=getVrepPolytope ( AHrep,bhrep,Qset );

    if ( !valid_poly ) {
        return 0.0;
    }
    getCartesianPolytope ( Qset,geometry_jacobians.back().topRows ( 3 ),geometry_transforms.back().translation(),Vset );
    if ( show_polytope ) {
        rvizDisplay.plotPolytope ( "polytope", Vset,geometry_transforms.back().translation(),color_pts, color_line );
    }

    vol_reduced=getPolytopeVolume ( Vset );
    return vol_reduced;
}


bool ConstrainedManipulability::getPolytopeHyperPlanes (
    const  KDL::JntArray & kdl_joint_positions,
    const  std::vector<shape_msgs::SolidPrimitive> &	geometry_mkrs,
    const TransformVector & geometry_transforms,
    const JacobianVector  & geometry_jacobians,
    Eigen::MatrixXd & AHrep,
    Eigen::VectorXd & bhrep,
    bool velocity_polytope
) {


    // Object ids
    std::vector<int> obj_ids;
    // Closest points
    std::vector<Eigen::Vector3d> p1w,p2w;
    // Min distance to object
    std::vector<double> obj_distances;
    Eigen::Vector3d nt;
    std::vector<double> distances;
    distances.clear();
    std::vector<Eigen::Matrix<double,1,Eigen::Dynamic>> J_constraints;
    J_constraints.clear();
    // For all robot links
    for ( int i = 0; i<geometry_mkrs.size(); i++ ) {
        fclInterface.checkDistanceObjectWorld ( geometry_mkrs[i],
                                                geometry_transforms[i],
                                                obj_ids,
                                                obj_distances,
                                                p1w,
                                                p2w );

        for ( unsigned int j=0
                             ; j<obj_distances.size(); j++ ) {
            Eigen::Matrix<double,6,Eigen::Dynamic> w_J_out_p1; //
            Eigen::Matrix<double,1,Eigen::Dynamic> J_proj;
            J_proj.setZero();
            w_J_out_p1.setZero();

            if ( obj_distances[j]<0.0 ) {
                // in collision
                ROS_WARN ( " In collision" );
                return false;
            } else if ( obj_distances[j]<distance_threshold_ ) {

                // Save the distance
                Eigen::Vector3d rdiff=p2w[j] - p1w[j];
                nt=rdiff; // direction of obstacle
                nt.normalize();
                // Get Jacobian at link
                Eigen::Vector3d w_delta_p1_collision_origin=p1w[j]-geometry_transforms[i].translation();
                // Get the Jacobian at p1

                screwTransform ( geometry_jacobians[i],
                                 w_J_out_p1,
                                 w_delta_p1_collision_origin );
                projectTranslationalJacobian ( nt,w_J_out_p1,J_proj );
                J_constraints.push_back ( J_proj );
                if ( velocity_polytope ) {
                    distances.push_back ( dangerfield_* ( obj_distances[j] *obj_distances[j] )-obj_distances[j] );
                } else {
                    distances.push_back ( obj_distances[j] );
                }
            }
        }
    }

    // For velocity polytope there are ndof*2 less hyperplanes
    int offset ( 4 );
    if ( velocity_polytope ) {
        offset=2;
    }
    AHrep.resize ( offset*ndof_+ J_constraints.size(),
                   ndof_ );
    bhrep.resize ( offset*ndof_ + distances.size(),1 );
    AHrep.setZero();
    bhrep.setZero();

    if ( velocity_polytope ) { // If velocity , then the joint position constraints simply scale max velocity
        AHrep.topRows ( ndof_ ) =ndof_identity_matrix_;  // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block ( ndof_,0,ndof_,ndof_ ) =-ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

        for ( int i = 0; i < ndof_; ++i ) {
            double qmean= ( qmax_[i]+qmin_[i] ) /2;
            double val_max=fmax ( qmean,kdl_joint_positions ( i ) )-qmean;
            double val_min=fmin ( qmean,kdl_joint_positions ( i ) )-qmean;
            double dmax=pow ( ( ( ( val_max ) / ( ( qmax_[i]-qmean ) ) ) ),2 );
            double dmin=pow ( ( ( ( val_min ) / ( ( qmin_[i]-qmean ) ) ) ),2 );
            // Make sure the value is witin joint limits and these limits are correctly defined.
            ROS_ASSERT ( ~std::isnan ( dmax ) && ~std::isnan ( dmin ) && ~std::isinf ( dmax ) && ~std::isinf ( dmin ) );
            bhrep ( i ) =dmax*qdotmax_[i];
            bhrep ( i+ndof_ ) =-dmin*qdotmin_[i];
        }

    } else {
        AHrep.topRows ( ndof_ ) =ndof_identity_matrix_;  // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block ( ndof_,0,ndof_,ndof_ ) =ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_
        AHrep.block ( ndof_*2,0,ndof_,ndof_ ) =ndof_identity_matrix_; // ndof_*ndof block at row  ndof_*2 colum 0;ndof_
        AHrep.block ( ndof_*3,0,ndof_,ndof_ ) =-ndof_identity_matrix_; // ndof_*ndof block at row  ndof_*3 colum 0;ndof_ -ndof_identity_matrix_ for negative joint limits
        for ( int i = 0; i < ndof_; ++i ) {
            bhrep ( i ) =qmax_[i]-kdl_joint_positions ( i );
            bhrep ( i+ndof_ ) =kdl_joint_positions ( i )-qmin_[i];
            bhrep ( i+2*ndof_ ) =max_lin_limit_[i];
            bhrep ( i+3*ndof_ ) =-min_lin_limit_[i];
        }
    }

    for ( int var = 0; var < J_constraints.size(); ++var ) {
        AHrep.row ( offset*ndof_+var ) =J_constraints[var];
        bhrep ( offset*ndof_+var ) =distances[var]; // Small tolerance to stop passing through
    }

    return true;
}




bool    ConstrainedManipulability::getCollisionModel ( const  KDL::JntArray & kdl_joint_positions,
        std::vector<shape_msgs::SolidPrimitive> & geometry_mkrs,
        TransformVector & geometry_transforms ) {

    geometry_mkrs.clear();
    geometry_transforms.clear();

    Eigen::Affine3d link_origin_T_collision_origin,base_T_link_origin,base_T_collision_origin;

    // Calculates the segement's collision geomtery
    //  The transform to the origin of the collision geometry
    //  The Jacobian matrix at the origin of the collision geometry
    for ( int i=0; i<chain_.getNrOfSegments(); ++i ) {
        KDL::Segment seg=chain_.getSegment ( i ); // Get current segment

        // Get Collision Geometry
        shape_msgs::SolidPrimitive s1;

        if ( model_.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::BOX ) {
            boost::shared_ptr<urdf::Box> box =
                boost::dynamic_pointer_cast<urdf::Box>
                ( model_.links_.at ( seg.getName() )->collision->geometry );

            s1.type=shape_msgs::SolidPrimitive::BOX;
            s1.dimensions.resize ( 3 );
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=box->dim.x;
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=box->dim.y;
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=box->dim.z;

        } else if ( model_.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::CYLINDER ) {
            boost::shared_ptr<urdf::Cylinder> cylinder =
                boost::dynamic_pointer_cast<urdf::Cylinder>
                ( model_.links_.at ( seg.getName() )->collision->geometry );

            s1.dimensions.resize ( 2 );
            s1.type=shape_msgs::SolidPrimitive::CYLINDER;
            s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=cylinder->length;
            s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=cylinder->radius;

        } else if ( model_.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::SPHERE ) {
            s1.type=shape_msgs::SolidPrimitive::SPHERE;
            boost::shared_ptr<urdf::Sphere> sphere =
                boost::dynamic_pointer_cast<urdf::Sphere>
                ( model_.links_.at ( seg.getName() )->collision->geometry );
            s1.dimensions.resize ( 1 );
            s1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=sphere->radius;
        } else if ( ( model_.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::MESH ) ) {

        } else {
            ROS_ERROR ( "Unsupported Collision Geometry" );
            return false;

        }


        // Get Collision Origin
        Eigen::Vector3d origin_Trans_collision ( model_.links_.at ( seg.getName() )->collision->origin.position.x,
                model_.links_.at ( seg.getName() )->collision->origin.position.y,
                model_.links_.at ( seg.getName() )->collision->origin.position.z );
        Eigen::Quaterniond origin_Quat_collision (
            model_.links_.at ( seg.getName() )->collision->origin.rotation.w,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.x,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.y,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.z
        );
        link_origin_T_collision_origin.translation() =origin_Trans_collision;
        link_origin_T_collision_origin.linear() =origin_Quat_collision.toRotationMatrix();
        // Finds cartesian pose w.r.t to base frame

        KDL::Frame cartpos;
        kdl_fk_solver_->JntToCart ( kdl_joint_positions,cartpos,i+1 );

        tf::transformKDLToEigen ( cartpos,base_T_link_origin );

        base_T_collision_origin=base_T_link_origin*link_origin_T_collision_origin;

        // Push back solutions
        geometry_mkrs.push_back ( s1 );
        geometry_transforms.push_back ( base_T_collision_origin );
    }
    return true;
}


std::unique_ptr<shapes::Shape> ConstrainedManipulability::constructShape ( const urdf::Geometry *geom ) {
    ROS_ASSERT ( geom );

    std::unique_ptr<shapes::Shape> result = NULL;
    switch ( geom->type ) {
    case urdf::Geometry::SPHERE: {
        result =  std::unique_ptr<shapes::Shape> ( new shapes::Sphere ( dynamic_cast<const urdf::Sphere*> ( geom )->radius ) );
        break;
    }
    case urdf::Geometry::BOX: {
        urdf::Vector3 dim = dynamic_cast<const urdf::Box*> ( geom )->dim;
        result =  std::unique_ptr<shapes::Shape> ( new shapes::Box ( dim.x, dim.y, dim.z ) );
        break;
    }
    case urdf::Geometry::CYLINDER: {
        result =  std::unique_ptr<shapes::Shape> ( new shapes::Cylinder ( dynamic_cast<const urdf::Cylinder*> ( geom )->radius,
                  dynamic_cast<const urdf::Cylinder*> ( geom )->length ) );
        break;
    }
    case urdf::Geometry::MESH: {
        const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*> ( geom );
        if ( !mesh->filename.empty() ) {
            Eigen::Vector3d scale ( mesh->scale.x, mesh->scale.y, mesh->scale.z );
            result =  std::unique_ptr<shapes::Shape> ( shapes::createMeshFromResource ( mesh->filename, scale ) );
        } else {
            ROS_WARN ( "Empty mesh filename" );
        }
        break;
    }
    default: {
        ROS_ERROR ( "Unknown geometry type: %d", ( int ) geom->type );
        break;
    }
    }
    return ( result );
}

bool ConstrainedManipulability::displayCollisionModel ( sensor_msgs::JointState const & joint_state ) {

    KDL::JntArray 	kdl_joint_positions ( ndof_ );
    jointStatetoKDLJointArray ( joint_state,kdl_joint_positions );
    std::vector<shape_msgs::SolidPrimitive> 	geometry_mkrs;
//     // Collision Link transforms
    TransformVector geometry_transforms;
    getCollisionModel ( kdl_joint_positions,geometry_mkrs,geometry_transforms );
    for ( int i=0; i<geometry_transforms.size(); ++i ) {
        rvizDisplay.displayMarker ( geometry_mkrs[i],geometry_transforms[i],base_link_,i, {0.1,0.5,0.2,0.5} );
    }
    return false;
}


bool ConstrainedManipulability::displayCollisionModel ( sensor_msgs::JointState const & joint_state, bool mesh ) {
    GeometryInformation geometry_information;
    KDL::JntArray 	kdl_joint_positions ( ndof_ );
    jointStatetoKDLJointArray ( joint_state,kdl_joint_positions );
    getCollisionModel ( kdl_joint_positions,geometry_information );

    for ( int i=0; i<geometry_information.geometry_transforms.size(); ++i ) {
        visualization_msgs::Marker mk;
        shapes::constructMarkerFromShape ( geometry_information.shapes[i].get(),mk, false );
        rvizDisplay.displayMarker ( mk,
                                    geometry_information.geometry_transforms[i],
                                    base_link_,i, {0.1,0.5,0.2,0.5} );
    }
    return true;

}

bool ConstrainedManipulability::getCollisionModel ( const  KDL::JntArray & kdl_joint_positions,
        GeometryInformation & geometry_information
                                                  ) {

    geometry_information.clear();
    Eigen::Affine3d link_origin_T_collision_origin,base_T_link_origin,base_T_collision_origin;

    // Calculates the segement's collision geomtery
    //  The transform to the origin of the collision geometry
    //  The Jacobian matrix at the origin of the collision geometry
    for ( int i=0; i<chain_.getNrOfSegments(); ++i ) {
        KDL::Segment seg=chain_.getSegment ( i ); // Get current segment

        // Get Collision Geometry
        std::unique_ptr<shapes::Shape> shape = constructShape ( model_.links_.at ( seg.getName() )->collision->geometry.get() );


        // Get Collision Origin
        Eigen::Vector3d origin_Trans_collision ( model_.links_.at ( seg.getName() )->collision->origin.position.x,
                model_.links_.at ( seg.getName() )->collision->origin.position.y,
                model_.links_.at ( seg.getName() )->collision->origin.position.z );
        Eigen::Quaterniond origin_Quat_collision (
            model_.links_.at ( seg.getName() )->collision->origin.rotation.w,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.x,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.y,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.z
        );

        link_origin_T_collision_origin.translation() =origin_Trans_collision;
        link_origin_T_collision_origin.linear() =origin_Quat_collision.toRotationMatrix();

        // Finds cartesian pose w.r.t to base frame
        KDL::Frame cartpos;
        kdl_fk_solver_->JntToCart ( kdl_joint_positions,cartpos,i+1 );

        tf::transformKDLToEigen ( cartpos,base_T_link_origin );

        base_T_collision_origin=base_T_link_origin*link_origin_T_collision_origin;

        // Get Jacobian at collision geometry origin
        KDL::Jacobian base_J_link_origin;
        base_J_link_origin.resize ( ndof_ );

        kdl_dfk_solver_->JntToJac ( kdl_joint_positions,base_J_link_origin,i+1 );
        Eigen::MatrixXd Jac=base_J_link_origin.data;

        Eigen::Vector3d base_L_link_collision= ( base_T_link_origin.linear() * link_origin_T_collision_origin.translation() );



        Eigen::Matrix<double,6,Eigen::Dynamic> base_J_collision_origin;

        screwTransform ( base_J_link_origin.data,base_J_collision_origin,base_L_link_collision );

        // Push back solutions
        geometry_information.shapes.push_back ( std::move ( shape ) );
        geometry_information.geometry_transforms.push_back ( base_T_collision_origin );
        geometry_information.geometry_jacobians.push_back ( base_J_collision_origin );
    }
    return true;



}


bool    ConstrainedManipulability::getCollisionModel ( const  KDL::JntArray & kdl_joint_positions,
        std::vector<shape_msgs::SolidPrimitive> & geometry_mkrs,
        TransformVector & geometry_transforms,
        JacobianVector  & geometry_jacobians ) {


    geometry_mkrs.clear();
    geometry_transforms.clear();
    geometry_jacobians.clear();


    Eigen::Affine3d link_origin_T_collision_origin,base_T_link_origin,base_T_collision_origin;

    // Calculates the segement's collision geomtery
    //  The transform to the origin of the collision geometry
    //  The Jacobian matrix at the origin of the collision geometry
    for ( int i=0; i<chain_.getNrOfSegments(); ++i ) {
        KDL::Segment seg=chain_.getSegment ( i ); // Get current segment

        // Get Collision Geometry
        shape_msgs::SolidPrimitive s1;

        if ( model_.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::BOX ) {
            boost::shared_ptr<urdf::Box> box =
                boost::dynamic_pointer_cast<urdf::Box>
                ( model_.links_.at ( seg.getName() )->collision->geometry );

            s1.type=shape_msgs::SolidPrimitive::BOX;
            s1.dimensions.resize ( 3 );
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=box->dim.x;
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=box->dim.y;
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=box->dim.z;

        } else if ( model_.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::CYLINDER ) {
            boost::shared_ptr<urdf::Cylinder> cylinder =
                boost::dynamic_pointer_cast<urdf::Cylinder>
                ( model_.links_.at ( seg.getName() )->collision->geometry );

            s1.dimensions.resize ( 2 );
            s1.type=shape_msgs::SolidPrimitive::CYLINDER;
            s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=cylinder->length;
            s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=cylinder->radius;

        } else if ( model_.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::SPHERE ) {
            s1.type=shape_msgs::SolidPrimitive::SPHERE;
            boost::shared_ptr<urdf::Sphere> sphere =
                boost::dynamic_pointer_cast<urdf::Sphere>
                ( model_.links_.at ( seg.getName() )->collision->geometry );
            s1.dimensions.resize ( 1 );
            s1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=sphere->radius;
        } else {
            ROS_ERROR ( "Unsupported Collision Geometry" );
            return false;

        }


        // Get Collision Origin
        Eigen::Vector3d origin_Trans_collision ( model_.links_.at ( seg.getName() )->collision->origin.position.x,
                model_.links_.at ( seg.getName() )->collision->origin.position.y,
                model_.links_.at ( seg.getName() )->collision->origin.position.z );
        Eigen::Quaterniond origin_Quat_collision (
            model_.links_.at ( seg.getName() )->collision->origin.rotation.w,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.x,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.y,
            model_.links_.at ( seg.getName() )->collision->origin.rotation.z
        );

        link_origin_T_collision_origin.translation() =origin_Trans_collision;
        link_origin_T_collision_origin.linear() =origin_Quat_collision.toRotationMatrix();
        // Finds cartesian pose w.r.t to base frame

        KDL::Frame cartpos;
        kdl_fk_solver_->JntToCart ( kdl_joint_positions,cartpos,i+1 );

        tf::transformKDLToEigen ( cartpos,base_T_link_origin );

        base_T_collision_origin=base_T_link_origin*link_origin_T_collision_origin;

        // Get Jacobian at collision geometry origin
        KDL::Jacobian base_J_link_origin;
        base_J_link_origin.resize ( ndof_ );

        kdl_dfk_solver_->JntToJac ( kdl_joint_positions,base_J_link_origin,i+1 );
        Eigen::MatrixXd Jac=base_J_link_origin.data;

        Eigen::Vector3d base_L_link_collision= ( base_T_link_origin.linear() * link_origin_T_collision_origin.translation() );



        Eigen::Matrix<double,6,Eigen::Dynamic> _base_J_collision_origin;

        screwTransform ( base_J_link_origin.data,_base_J_collision_origin,base_L_link_collision );

        // Push back solutions
        geometry_mkrs.push_back ( s1 );
        geometry_transforms.push_back ( base_T_collision_origin );
        geometry_jacobians.push_back ( _base_J_collision_origin );
    }
    return true;
}


bool ConstrainedManipulability::projectTranslationalJacobian ( Eigen::Vector3d nT,
        const Eigen::Matrix<double,6,Eigen::Dynamic>& J0N_in,
        Eigen::Matrix<double,1,Eigen::Dynamic>& J0N_out ) {

    int n=J0N_in.cols();
    ROS_ASSERT ( J0N_in.rows() >3 );
    J0N_out.setZero();
    J0N_out=nT.transpose() *J0N_in.topLeftCorner ( 3,n );
}


bool ConstrainedManipulability::screwTransform ( const Eigen::Matrix<double,6,Eigen::Dynamic> &J0N_in,
        Eigen::Matrix<double,6,Eigen::Dynamic>& J0E_out,
        const Eigen::Vector3d & L ) {
    J0E_out.setZero();
    Eigen::Matrix<double,3,3> Lhat;
    Lhat.setZero();
    skew ( L,Lhat );
    Eigen::Matrix<double,6,6> screwL;
    screwL.setZero();
    screwL.topLeftCorner ( 3,3 ) =Eigen::Matrix<double,3,3>::Identity();
    screwL.bottomRightCorner ( 3,3 ) =Eigen::Matrix<double,3,3>::Identity();
    screwL.bottomLeftCorner ( 3,3 ) =Eigen::Matrix<double,3,3>::Zero();
    screwL.topRightCorner ( 3,3 ) =-Lhat;
    J0E_out=screwL*J0N_in;
    return true;
}






bool ConstrainedManipulability::getVrepPolytope ( const Eigen::MatrixXd & A_left,
        const Eigen::VectorXd & b_left,
        Eigen::MatrixXd& reduced_joint_vertex_set ) {

    Eigen::Polyhedron Poly;
    try {
        Poly.setHrep ( A_left,b_left );
        auto vrep=Poly.vrep();
        reduced_joint_vertex_set=vrep.first;
        if ( reduced_joint_vertex_set.rows() <=0 ) {
            ROS_ERROR ( "V representation error no rows" );
            return false;
        }
    } catch ( ... ) {
        ROS_ERROR ( "V representation error" );
        return false;
    }
    return true;
}

bool ConstrainedManipulability::getHrepPolytope ( const Eigen::MatrixXd& vertex_set,
        Eigen::MatrixXd& A_left,
        Eigen::VectorXd& b_left ) {

    Eigen::Polyhedron Poly;
    Eigen::VectorXd v_type; // v_type indicates vertices or rays in polytope definition
    v_type.resize ( vertex_set.rows() );
    v_type.setOnes(); // Ones for vertices and zeros for rays

    try {
        Poly.setVertices ( vertex_set );
        auto hrep=Poly.hrep();
        A_left=hrep.first;
        b_left=hrep.second;
    } catch ( ... ) {
        ROS_ERROR ( "H representation error" );
        return false;
    }
    return true;
}



void ConstrainedManipulability::getCartesianPolytope ( Eigen::MatrixXd Q,
        Eigen::Matrix<double,3,7> Jp,
        Eigen::Vector3d P,
        Eigen::MatrixXd& V ) {
    V.resize ( Q.rows(),3 );
    V.setZero();
    transformVertexSet ( Jp,Q,V );
}


void ConstrainedManipulability::transformVertexSet ( Eigen::Matrix<double,3,7> J,
        const Eigen::MatrixXd & vertex_set,
        Eigen::MatrixXd & vertex_set_out ) {
    for ( int var = 0; var < vertex_set.rows(); ++var ) {
        Eigen::VectorXd vertex=vertex_set.row ( var );
        Eigen::VectorXd p=J*vertex;
        vertex_set_out.row ( var ) =p;
    }
}

ConstrainedManipulability::~ConstrainedManipulability() {
    ROS_INFO ( "Calling Destructor" );
}



void ConstrainedManipulability::jointStatetoKDLJointArray ( const sensor_msgs::JointState & joint_states,
        KDL::JntArray & kdl_joint_positions ) {
    unsigned int jnt ( 0 );
    for ( int i=0; i<chain_.getNrOfSegments(); ++i ) {

        KDL::Segment seg=chain_.getSegment ( i );
        KDL::Joint kdl_joint=seg.getJoint();
        for ( int j=0; j<joint_states.name.size(); ++j ) {
            if ( kdl_joint.getName() ==joint_states.name[j] ) {
                kdl_joint_positions ( jnt ) =joint_states.position[j];
                jnt++;
            }
        }
    }
}

// --------- STATIC FUNCTIONS FOR OPTIMIZATION-------------------------



void ConstrainedManipulability::jointStatetoKDLJointArray ( KDL::Chain & chain, const sensor_msgs::JointState & joint_states,
        KDL::JntArray & kdl_joint_positions ) {
    unsigned int jnt ( 0 );
    for ( int i=0; i<chain.getNrOfSegments(); ++i ) {

        KDL::Segment seg=chain.getSegment ( i );
        KDL::Joint kdl_joint=seg.getJoint();
        for ( int j=0; j<joint_states.name.size(); ++j ) {
            if ( kdl_joint.getName() ==joint_states.name[j] ) {
                kdl_joint_positions ( jnt ) =joint_states.position[j];
                jnt++;
            }
        }
    }
}


double ConstrainedManipulability::getConstrainedAllowableMotionPolytope ( KDL::Chain &  chain,
        urdf::Model & model,
        FCLObjectSet objects,
        const sensor_msgs::JointState & joint_states,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        double linearization_limit,
        double distance_threshold ) {

    int ndof=chain.getNrOfJoints();

    Eigen::MatrixXd Qset,Vset;

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize ( ndof,ndof );
    ndof_identity_matrix.setZero();
    for ( int i = 0; i<ndof_identity_matrix.rows(); i++ ) {
        for ( int j = 0; j<ndof_identity_matrix.cols(); j++ ) {
            if ( i==j ) {
                ndof_identity_matrix ( i,j ) =1;
            }
        }
    }

    KDL::JntArray 	kdl_joint_positions ( ndof );
    jointStatetoKDLJointArray ( chain,joint_states,kdl_joint_positions );

    boost::scoped_ptr<KDL::ChainJntToJacSolver>  kdl_dfk_solver;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver;


    //     // Collision model
    std::vector<shape_msgs::SolidPrimitive> 	geometry_mkrs;
//     // Collision Link transforms
    TransformVector geometry_transforms;
//     // Collision Link Jacobians
    JacobianVector geometry_jacobians;


    getCollisionModel ( chain,model, kdl_joint_positions,geometry_mkrs,geometry_transforms,geometry_jacobians );


    bool collision_free=getPolytopeHyperPlanes ( chain,
                        model,
                        objects,
                        kdl_joint_positions,
                        geometry_mkrs,
                        geometry_transforms,
                        geometry_jacobians ,
                        AHrep,
                        bhrep,
                        distance_threshold,
                        linearization_limit );

    if ( !collision_free ) {
        return 0.0;
    }


    bool valid_poly=getVrepPolytope ( AHrep,bhrep,Qset );

    if ( !valid_poly ) {
        return 0.0;
    }
    getCartesianPolytope ( Qset,geometry_jacobians.back().topRows ( 3 ),geometry_transforms.back().translation(),Vset );

    double vol_reduced=getPolytopeVolume ( Vset );
    return vol_reduced;

}




double ConstrainedManipulability::getConstrainedVelocityPolytope ( KDL::Chain &  chain,
        urdf::Model & model,
        FCLObjectSet objects,
        const sensor_msgs::JointState & joint_states,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        double dangerfield,
        double distance_threshold ) {

    int ndof=chain.getNrOfJoints();

    Eigen::MatrixXd Qset,Vset;

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize ( ndof,ndof );
    ndof_identity_matrix.setZero();
    for ( int i = 0; i<ndof_identity_matrix.rows(); i++ ) {
        for ( int j = 0; j<ndof_identity_matrix.cols(); j++ ) {
            if ( i==j ) {
                ndof_identity_matrix ( i,j ) =1;
            }
        }
    }

    KDL::JntArray 	kdl_joint_positions ( ndof );
    jointStatetoKDLJointArray ( chain,joint_states,kdl_joint_positions );

    boost::scoped_ptr<KDL::ChainJntToJacSolver>  kdl_dfk_solver;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver;


    //     // Collision model
    std::vector<shape_msgs::SolidPrimitive> 	geometry_mkrs;
//     // Collision Link transforms
    TransformVector geometry_transforms;
//     // Collision Link Jacobians
    JacobianVector geometry_jacobians;


    getCollisionModel ( chain,model, kdl_joint_positions,geometry_mkrs,geometry_transforms,geometry_jacobians );


    bool collision_free=getPolytopeHyperPlanes ( chain,
                        model,
                        objects,
                        kdl_joint_positions,
                        geometry_mkrs,
                        geometry_transforms,
                        geometry_jacobians ,
                        AHrep,
                        bhrep,
                        distance_threshold,
                        0.0,
                        true,dangerfield );

    if ( !collision_free ) {
        return 0.0;
    }


    bool valid_poly=getVrepPolytope ( AHrep,bhrep,Qset );

    if ( !valid_poly ) {
        return 0.0;
    }
    getCartesianPolytope ( Qset,geometry_jacobians.back().topRows ( 3 ),geometry_transforms.back().translation(),Vset );

    double vol_reduced=getPolytopeVolume ( Vset );
    return vol_reduced;

}



bool ConstrainedManipulability::getPolytopeHyperPlanes ( KDL::Chain &  chain,
        urdf::Model & model,
        FCLObjectSet objects,
        const  KDL::JntArray & kdl_joint_positions,
        const  std::vector<shape_msgs::SolidPrimitive> &	geometry_mkrs,
        const TransformVector & geometry_transforms,
        const JacobianVector  & geometry_jacobians,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        double distance_threshold,
        double linearization_limit,
        bool velocity_polytope,
        double dangerfield
                                                       ) {

    int ndof=chain.getNrOfJoints();
    std::vector<double> qmax ( ndof ),qmin ( ndof ),qdotmax ( ndof ),qdotmin ( ndof );

    int mvable_jnt ( 0 );
    for ( int i=0; i<ndof+1; ++i ) {
        KDL::Segment seg=chain.getSegment ( i );
        KDL::Joint kdl_joint=seg.getJoint();

        if ( kdl_joint.getType() !=KDL::Joint::None ) {
            qmax[mvable_jnt]=model.joints_.at ( kdl_joint.getName() )->limits->upper;
            qmin[mvable_jnt]=model.joints_.at ( kdl_joint.getName() )->limits->lower;

            qdotmax[mvable_jnt]=model.joints_.at ( kdl_joint.getName() )->limits->velocity;
            qdotmin[mvable_jnt]=-model.joints_.at ( kdl_joint.getName() )->limits->velocity;


            mvable_jnt++;
        }

    }

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize ( ndof,ndof );
    ndof_identity_matrix.setZero();
    for ( int i = 0; i<ndof_identity_matrix.rows(); i++ ) {
        for ( int j = 0; j<ndof_identity_matrix.cols(); j++ ) {
            if ( i==j ) {
                ndof_identity_matrix ( i,j ) =1;
            }
        }
    }
    // Object ids
    std::vector<int> obj_ids;
    // Closest points
    std::vector<Eigen::Vector3d> p1w,p2w;
    // Min distance to object
    std::vector<double> obj_distances;
    Eigen::Vector3d nt;
    std::vector<double> distances;
    distances.clear();
    std::vector<Eigen::Matrix<double,1,Eigen::Dynamic>> J_constraints;
    J_constraints.clear();
    // For all robot links
    for ( int i = 0; i<geometry_mkrs.size(); i++ ) {

        FCLInterface::checkDistanceObjectWorld ( geometry_mkrs[i],
                geometry_transforms[i],
                objects,
                obj_distances,
                p1w,
                p2w );

        for ( unsigned int j=0
                             ; j<obj_distances.size(); j++ ) {
            Eigen::Matrix<double,6,Eigen::Dynamic> w_J_out_p1; //
            Eigen::Matrix<double,1,Eigen::Dynamic> J_proj;
            J_proj.setZero();
            w_J_out_p1.setZero();

            if ( obj_distances[j]<0.0 ) {
                // in collision
                ROS_WARN ( " In collision" );
                return false;
            } else if ( obj_distances[j]<distance_threshold ) {

                // Save the distance
                Eigen::Vector3d rdiff=p2w[j] - p1w[j];
                nt=rdiff; // direction of obstacle
                nt.normalize();
                // Get Jacobian at link
                Eigen::Vector3d w_delta_p1_collision_origin=p1w[j]-geometry_transforms[i].translation();
                // Get the Jacobian at p1

                screwTransform ( geometry_jacobians[i],
                                 w_J_out_p1,
                                 w_delta_p1_collision_origin );
                projectTranslationalJacobian ( nt,w_J_out_p1,J_proj );
                J_constraints.push_back ( J_proj );
                if ( velocity_polytope ) {
                    distances.push_back ( dangerfield* ( obj_distances[j] *obj_distances[j] )-obj_distances[j] );
                } else {
                    distances.push_back ( obj_distances[j] );
                }
            }
        }
    }

    // For velocity polytope there are ndof*2 less hyperplanes
    int offset ( 4 );
    if ( velocity_polytope ) {
        offset=2;
    }
    AHrep.resize ( offset*ndof+ J_constraints.size(),
                   ndof );
    bhrep.resize ( offset*ndof + distances.size(),1 );
    AHrep.setZero();
    bhrep.setZero();

    if ( velocity_polytope ) { // If velocity , then the joint position constraints simply scale max velocity
        AHrep.topRows ( ndof ) =ndof_identity_matrix;  // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block ( ndof,0,ndof,ndof ) =-ndof_identity_matrix; // ndof_*ndof block at row  ndof_ colum 0;ndof_

        for ( int i = 0; i < ndof; ++i ) {
            double qmean= ( qmax[i]+qmin[i] ) /2;
            double val_max=fmax ( qmean,kdl_joint_positions ( i ) )-qmean;
            double val_min=fmin ( qmean,kdl_joint_positions ( i ) )-qmean;
            double dmax=pow ( ( ( ( val_max ) / ( ( qmax[i]-qmean ) ) ) ),2 );
            double dmin=pow ( ( ( ( val_min ) / ( ( qmin[i]-qmean ) ) ) ),2 );
            // Make sure the value is witin joint limits and these limits are correctly defined.
            ROS_ASSERT ( ~std::isnan ( dmax ) && ~std::isnan ( dmin ) && ~std::isinf ( dmax ) && ~std::isinf ( dmin ) );
            bhrep ( i ) =dmax*qdotmax[i];
            bhrep ( i+ndof ) =-dmin*qdotmin[i];
        }

    } else {

        AHrep.topRows ( ndof ) =ndof_identity_matrix;  // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block ( ndof,0,ndof,ndof ) =ndof_identity_matrix; // ndof_*ndof block at row  ndof_ colum 0;ndof_
        AHrep.block ( ndof*2,0,ndof,ndof ) =ndof_identity_matrix; // ndof_*ndof block at row  ndof_*2 colum 0;ndof_
        AHrep.block ( ndof*3,0,ndof,ndof ) =-ndof_identity_matrix; // ndof_*ndof block at row  ndof_*3 colum 0;ndof_ -ndof_identity_matrix_ for negative joint limits
        for ( int i = 0; i < ndof; ++i ) {
            bhrep ( i ) =qmax[i]-kdl_joint_positions ( i );
            bhrep ( i+ndof ) =kdl_joint_positions ( i )-qmin[i];
            bhrep ( i+2*ndof ) =linearization_limit;
            bhrep ( i+3*ndof ) =-linearization_limit;
        }
    }

    for ( int var = 0; var < J_constraints.size(); ++var ) {
        AHrep.row ( offset*ndof+var ) =J_constraints[var];
        bhrep ( offset*ndof+var ) =distances[var]; // Small tolerance to stop passing through
    }

    return true;
}




bool    ConstrainedManipulability::getCollisionModel ( KDL::Chain &  chain,
        urdf::Model & model,
        const  KDL::JntArray & kdl_joint_positions,
        std::vector<shape_msgs::SolidPrimitive> & geometry_mkrs,
        TransformVector & geometry_transforms,
        JacobianVector  & geometry_jacobians
                                                     ) {




    geometry_mkrs.clear();
    geometry_transforms.clear();
    geometry_jacobians.clear();
    boost::scoped_ptr<KDL::ChainJntToJacSolver>  kdl_dfk_solver;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver;
    Eigen::Affine3d link_origin_T_collision_origin,base_T_link_origin,base_T_collision_origin;

    int ndof=chain.getNrOfJoints();

    // Calculates the segement's collision geomtery
    //  The transform to the origin of the collision geometry
    //  The Jacobian matrix at the origin of the collisiNon geometry
    for ( int i=0; i<chain.getNrOfSegments(); ++i ) {
        KDL::Segment seg=chain.getSegment ( i ); // Get current segment

        // Get Collision Geometry
        shape_msgs::SolidPrimitive s1;

        if ( model.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::BOX ) {
            boost::shared_ptr<urdf::Box> box =
                boost::dynamic_pointer_cast<urdf::Box>
                ( model.links_.at ( seg.getName() )->collision->geometry );

            s1.type=shape_msgs::SolidPrimitive::BOX;
            s1.dimensions.resize ( 3 );
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=box->dim.x;
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=box->dim.y;
            s1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=box->dim.z;

        } else if ( model.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::CYLINDER ) {
            boost::shared_ptr<urdf::Cylinder> cylinder =
                boost::dynamic_pointer_cast<urdf::Cylinder>
                ( model.links_.at ( seg.getName() )->collision->geometry );

            s1.dimensions.resize ( 2 );
            s1.type=shape_msgs::SolidPrimitive::CYLINDER;
            s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=cylinder->length;
            s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=cylinder->radius;

        } else if ( model.links_.at ( seg.getName() )->collision->geometry->type==urdf::Geometry::SPHERE ) {
            s1.type=shape_msgs::SolidPrimitive::SPHERE;
            boost::shared_ptr<urdf::Sphere> sphere =
                boost::dynamic_pointer_cast<urdf::Sphere>
                ( model.links_.at ( seg.getName() )->collision->geometry );
            s1.dimensions.resize ( 1 );
            s1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=sphere->radius;
        } else {
            ROS_ERROR ( "Unsupported Collision Geometry" );
            return false;

        }


        // Get Collision Origin
        Eigen::Vector3d origin_Trans_collision ( model.links_.at ( seg.getName() )->collision->origin.position.x,
                model.links_.at ( seg.getName() )->collision->origin.position.y,
                model.links_.at ( seg.getName() )->collision->origin.position.z );
        Eigen::Quaterniond origin_Quat_collision (
            model.links_.at ( seg.getName() )->collision->origin.rotation.w,
            model.links_.at ( seg.getName() )->collision->origin.rotation.x,
            model.links_.at ( seg.getName() )->collision->origin.rotation.y,
            model.links_.at ( seg.getName() )->collision->origin.rotation.z
        );

        link_origin_T_collision_origin.translation() =origin_Trans_collision;
        link_origin_T_collision_origin.linear() =origin_Quat_collision.toRotationMatrix();

        // Finds cartesian pose w.r.t to base frame

        KDL::Frame cartpos;
        kdl_fk_solver->JntToCart ( kdl_joint_positions,cartpos,i+1 );

        tf::transformKDLToEigen ( cartpos,base_T_link_origin );

        base_T_collision_origin=base_T_link_origin*link_origin_T_collision_origin;

        // Get Jacobian at collision geometry origin
        KDL::Jacobian base_J_link_origin;
        base_J_link_origin.resize ( ndof );

        kdl_dfk_solver->JntToJac ( kdl_joint_positions,base_J_link_origin,i+1 );
        Eigen::MatrixXd Jac=base_J_link_origin.data;

        Eigen::Vector3d base_L_link_collision= ( base_T_link_origin.linear() * link_origin_T_collision_origin.translation() );



        Eigen::Matrix<double,6,Eigen::Dynamic> _base_J_collision_origin;

        screwTransform ( base_J_link_origin.data,_base_J_collision_origin,base_L_link_collision );

        // Push back solutions
        geometry_mkrs.push_back ( s1 );
        geometry_transforms.push_back ( base_T_collision_origin );
        geometry_jacobians.push_back ( _base_J_collision_origin );
    }
    return true;
}






double ConstrainedManipulability::getAllowableMotionPolytope ( KDL::Chain &  chain,
        urdf::Model & model,
        const sensor_msgs::JointState & joint_states,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        double linearization_limit
                                                             ) {



    int ndof=chain.getNrOfJoints();

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize ( ndof,ndof );
    ndof_identity_matrix.setZero();
    for ( int i = 0; i<ndof_identity_matrix.rows(); i++ ) {
        for ( int j = 0; j<ndof_identity_matrix.cols(); j++ ) {
            if ( i==j ) {
                ndof_identity_matrix ( i,j ) =1;
            }
        }
    }


    KDL::JntArray 	kdl_joint_positions ( ndof );
    jointStatetoKDLJointArray ( chain,joint_states,kdl_joint_positions );

    boost::scoped_ptr<KDL::ChainJntToJacSolver>  kdl_dfk_solver;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver;

    std::vector<double> max_lin_limit ( ndof );
    std::vector<double> min_lin_limit ( ndof );

    std::fill ( max_lin_limit.begin(), max_lin_limit.end(),linearization_limit );
    std::fill ( min_lin_limit.begin(), min_lin_limit.end(),-linearization_limit );

    Eigen::Affine3d base_T_ee;
    KDL::Frame cartpos;
    KDL::Jacobian base_J_link_origin;
    base_J_link_origin.resize ( ndof );
    kdl_fk_solver->JntToCart ( kdl_joint_positions,cartpos );
    tf::transformKDLToEigen ( cartpos,base_T_ee );
    kdl_dfk_solver->JntToJac ( kdl_joint_positions,base_J_link_origin );
    Eigen::MatrixXd base_J_ee=base_J_link_origin.data;

    Eigen::MatrixXd Ahrep_undeformed;
    Eigen::VectorXd bhrep_underformed;
    Eigen::MatrixXd Qset_undeformed,Vset_undeformed;
    Ahrep_undeformed.resize ( 2*ndof,ndof );
    Ahrep_undeformed.topRows ( ndof ) =ndof_identity_matrix;  // ndof_*ndof block at row  0 colum 0;ndof_
    Ahrep_undeformed.block ( ndof,0,ndof,ndof ) =-ndof_identity_matrix; // ndof_*ndof block at row  ndof_ colum 0;ndof_
    bhrep_underformed.resize ( 2*ndof,1 );
    for ( int i = 0; i < ndof; ++i ) {
        bhrep_underformed ( i ) =max_lin_limit[i];
        bhrep_underformed ( i+ndof ) =-min_lin_limit[i];
    }


    getVrepPolytope ( Ahrep_undeformed,bhrep_underformed,Qset_undeformed );
    getCartesianPolytope ( Qset_undeformed,base_J_ee,base_T_ee.translation(),Vset_undeformed );
    double vol_initial=getPolytopeVolume ( Vset_undeformed );
    return vol_initial;
}

double ConstrainedManipulability::getVelocityPolytope ( KDL::Chain &  chain,
        urdf::Model & model,
        const sensor_msgs::JointState & joint_states,
        Eigen::MatrixXd & AHrep,
        Eigen::VectorXd & bhrep,
        std::vector<double> qdot_max,
        std::vector<double> qdot_min ) {

    int ndof=chain.getNrOfJoints();
    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize ( ndof,ndof );
    ndof_identity_matrix.setZero();
    for ( int i = 0; i<ndof_identity_matrix.rows(); i++ ) {
        for ( int j = 0; j<ndof_identity_matrix.cols(); j++ ) {
            if ( i==j ) {
                ndof_identity_matrix ( i,j ) =1;
            }
        }
    }


    KDL::JntArray 	kdl_joint_positions ( ndof );
    jointStatetoKDLJointArray ( chain,joint_states,kdl_joint_positions );

    boost::scoped_ptr<KDL::ChainJntToJacSolver>  kdl_dfk_solver;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver;

    std::vector<double> max_lin_limit ( ndof );
    std::vector<double> min_lin_limit ( ndof );


    Eigen::Affine3d base_T_ee;
    KDL::Frame cartpos;
    KDL::Jacobian base_J_link_origin;
    base_J_link_origin.resize ( ndof );
    kdl_fk_solver->JntToCart ( kdl_joint_positions,cartpos );
    tf::transformKDLToEigen ( cartpos,base_T_ee );
    kdl_dfk_solver->JntToJac ( kdl_joint_positions,base_J_link_origin );
    Eigen::MatrixXd base_J_ee=base_J_link_origin.data;

    Eigen::MatrixXd Ahrep_undeformed;
    Eigen::VectorXd bhrep_underformed;
    Eigen::MatrixXd Qset_undeformed,Vset_undeformed;
    Ahrep_undeformed.resize ( 2*ndof,ndof );
    Ahrep_undeformed.topRows ( ndof ) =ndof_identity_matrix;  // ndof_*ndof block at row  0 colum 0;ndof_
    Ahrep_undeformed.block ( ndof,0,ndof,ndof ) =-ndof_identity_matrix; // ndof_*ndof block at row  ndof_ colum 0;ndof_
    bhrep_underformed.resize ( 2*ndof,1 );
    for ( int i = 0; i < ndof; ++i ) {
        bhrep_underformed ( i ) =qdot_max[i];
        bhrep_underformed ( i+ndof ) =-qdot_min[i];
    }


    getVrepPolytope ( Ahrep_undeformed,bhrep_underformed,Qset_undeformed );
    getCartesianPolytope ( Qset_undeformed,base_J_ee,base_T_ee.translation(),Vset_undeformed );
    double vol_initial=getPolytopeVolume ( Vset_undeformed );
    return vol_initial;
}
