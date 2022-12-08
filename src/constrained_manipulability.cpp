#include <constrained_manipulability/ObjectDistances.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <constrained_manipulability/constrained_manipulability.hpp>

namespace constrained_manipulability
{
    ConstrainedManipulability::ConstrainedManipulability(ros::NodeHandle nh, std::string root, std::string tip, std::string robot_description,
                                                         double distance_threshold, double linearization_limit, double dangerfield)
        : nh_(nh), fclInterface(nh), distance_threshold_(distance_threshold), dangerfield_(dangerfield), octomap_id_(-1)
    {
        polytope_server_ = nh_.advertiseService("get_polytope_constraints", &ConstrainedManipulability::getPolytopeConstraintsCallback, this);
        jacobian_server_ = nh_.advertiseService("get_jacobian_matrix", &ConstrainedManipulability::getJacobianCallback, this);

        obj_dist_pub_ = nh_.advertise<constrained_manipulability::ObjectDistances>("constrained_manipulability/obj_distances", 1);
        mkr_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

        octo_filter_ = new octomap_filter::OctomapFilter(nh_);

        std::string robot_desc_string;
        nh_.param(robot_description, robot_desc_string, std::string());
        model_.initParamWithNodeHandle(robot_description, nh);

        if (!kdl_parser::treeFromString(robot_desc_string, my_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
        }

        base_link_ = root;
        my_tree_.getChain(root,
                          tip,
                          chain_);

        ndof_ = chain_.getNrOfJoints();

        ROS_INFO_STREAM("Loading tree from parameter " << robot_description << " with kinematic chain from " << root << " to " << tip);
        ROS_INFO_STREAM("Number of tree segments_= " << my_tree_.getNrOfSegments());
        ROS_INFO_STREAM("Number of tree joints= " << my_tree_.getNrOfJoints());
        ROS_INFO_STREAM("Number of chain joints= " << chain_.getNrOfJoints());
        ROS_INFO_STREAM("Number of chain segments= " << chain_.getNrOfSegments());

        if (ndof_ < 1)
        {
            ROS_FATAL("Robot has 0 joints, check if root and/or tip name is incorrect");
            ros::shutdown();
        }

        qmax_.resize(ndof_);
        qmin_.resize(ndof_);
        qdotmax_.resize(ndof_);
        qdotmin_.resize(ndof_);
        max_lin_limit_.resize(ndof_);
        min_lin_limit_.resize(ndof_);
        // Default values

        kdl_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
        kdl_dfk_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

        int mvable_jnt(0);

        std::vector<std::string> joint_names(ndof_);
        for (int i = 0; i < chain_.getNrOfSegments(); ++i)
        {

            KDL::Segment seg = chain_.getSegment(i);
            KDL::Joint kdl_joint = seg.getJoint();
            urdf::JointConstSharedPtr urdf_joint = model_.getJoint(kdl_joint.getName());

            if (kdl_joint.getType() != KDL::Joint::None)
            {
                // No limits so assign max
                if (urdf_joint->type == urdf::Joint::CONTINUOUS)
                {
                    qmax_[mvable_jnt] = 2.0 * M_PI;
                    qmin_[mvable_jnt] = -2.0 * M_PI;
                }
                else
                {
                    qmax_[mvable_jnt] = model_.joints_.at(kdl_joint.getName())->limits->upper;
                    qmin_[mvable_jnt] = model_.joints_.at(kdl_joint.getName())->limits->lower;
                }

                qdotmax_[mvable_jnt] = model_.joints_.at(kdl_joint.getName())->limits->velocity;
                qdotmin_[mvable_jnt] = -model_.joints_.at(kdl_joint.getName())->limits->velocity;
                joint_names[mvable_jnt] = kdl_joint.getName();
                mvable_jnt++;
            }
        }

        nh.setParam("constrained_manipulability/active_dof", joint_names);

        ndof_identity_matrix_.resize(ndof_, ndof_);
        ndof_identity_matrix_.setZero();
        for (int i = 0; i < ndof_identity_matrix_.rows(); i++)
        {
            for (int j = 0; j < ndof_identity_matrix_.cols(); j++)
            {
                if (i == j)
                {
                    ndof_identity_matrix_(i, j) = 1;
                }
            }
        }

        constrained_manipulability::printVector(qmax_, "qmax_");
        constrained_manipulability::printVector(qmin_, "qmin_");
        constrained_manipulability::printVector(qdotmax_, "qdotmax_");
        constrained_manipulability::printVector(qdotmin_, "qdotmin_");

        std::fill(max_lin_limit_.begin(), max_lin_limit_.end(), linearization_limit);
        std::fill(min_lin_limit_.begin(), min_lin_limit_.end(), -linearization_limit);

        ROS_INFO("Initialized");
    }

    ConstrainedManipulability::~ConstrainedManipulability()
    {
        ROS_INFO("Calling Destructor");
        if (octo_filter_ != NULL)
        {
            delete octo_filter_;
        }
    }

    Polytope ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                   bool show_polytope,
                                                                   std::vector<double> color_pts,
                                                                   std::vector<double> color_line)
    {
        Eigen::MatrixXd AHrep;
        Eigen::VectorXd bhrep;
        Eigen::Vector3d offset_position;

        Polytope poly = getAllowableMotionPolytope(joint_states,
                                                   AHrep,
                                                   bhrep,
                                                   offset_position,
                                                   show_polytope,
                                                   color_pts,
                                                   color_line);
        return poly;
    }

    Polytope ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                   Eigen::MatrixXd &AHrep,
                                                                   Eigen::VectorXd &bhrep,
                                                                   bool show_polytope,
                                                                   std::vector<double> color_pts,
                                                                   std::vector<double> color_line)
    {
        Eigen::Vector3d offset_position;

        Polytope poly = getAllowableMotionPolytope(joint_states,
                                                   AHrep,
                                                   bhrep,
                                                   offset_position,
                                                   show_polytope,
                                                   color_pts,
                                                   color_line);
        return poly;
    }

    Polytope ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                   Eigen::MatrixXd &AHrep,
                                                                   Eigen::VectorXd &bhrep,
                                                                   Eigen::Vector3d &offset_position,
                                                                   bool show_polytope,
                                                                   std::vector<double> color_pts,
                                                                   std::vector<double> color_line)
    {
        Eigen::Affine3d base_T_ee;
        Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
        KDL::JntArray kdl_joint_positions(ndof_);

        jointStatetoKDLJointArray(joint_states, kdl_joint_positions);
        getKDLKinematicInformation(kdl_joint_positions, base_T_ee, base_J_ee);

        // Define Hyperplanes
        AHrep.resize(2 * ndof_, ndof_);
        AHrep.topRows(ndof_) = ndof_identity_matrix_;                 // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

        // Define shifted distance from origin
        bhrep.resize(2 * ndof_, 1);
        for (int i = 0; i < ndof_; ++i)
        {
            bhrep(i) = max_lin_limit_[i];
            bhrep(i + ndof_) = -min_lin_limit_[i];
        }

        // Define offset position to end effector
        offset_position = base_T_ee.translation();

        // Convert to V-representation polytope
        Polytope vrep_polytope(nh_, "allowable_motion_polytope", base_link_, AHrep, bhrep);
        vrep_polytope.transformCartesian(base_J_ee.topRows(3), offset_position);

        if (show_polytope)
        {
            vrep_polytope.plot(offset_position, color_pts, color_line);
        }

        return vrep_polytope;
    }

    Polytope ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                              bool show_polytope,
                                                                              std::vector<double> color_pts,
                                                                              std::vector<double> color_line)
    {
        Eigen::MatrixXd AHrep;
        Eigen::VectorXd bhrep;
        Eigen::Vector3d offset_position;

        Polytope poly = getConstrainedAllowableMotionPolytope(joint_states,
                                                              AHrep,
                                                              bhrep,
                                                              offset_position,
                                                              show_polytope,
                                                              color_pts,
                                                              color_line);

        return poly;
    }

    Polytope ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                              Eigen::MatrixXd &AHrep,
                                                                              Eigen::VectorXd &bhrep,
                                                                              bool show_polytope,
                                                                              std::vector<double> color_pts,
                                                                              std::vector<double> color_line)
    {
        Eigen::Vector3d offset_position;

        Polytope poly = getConstrainedAllowableMotionPolytope(joint_states,
                                                              AHrep,
                                                              bhrep,
                                                              offset_position,
                                                              show_polytope,
                                                              color_pts,
                                                              color_line);
        return poly;
    }

    Polytope ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                              Eigen::MatrixXd &AHrep,
                                                                              Eigen::VectorXd &bhrep,
                                                                              Eigen::Vector3d &offset_position,
                                                                              bool show_polytope,
                                                                              std::vector<double> color_pts,
                                                                              std::vector<double> color_line)
    {
        GeometryInformation geometry_information;
        KDL::JntArray kdl_joint_positions(ndof_);

        jointStatetoKDLJointArray(joint_states, kdl_joint_positions);
        getCollisionModel(kdl_joint_positions, geometry_information);

        bool collision_free = getPolytopeHyperPlanes(kdl_joint_positions,
                                                     geometry_information,
                                                     AHrep,
                                                     bhrep);

        if (!collision_free)
        {
            return Polytope();
        }

        // Define offset position based on geometrical collision world for robot body
        offset_position = geometry_information.geometry_transforms.back().translation();

        // Convert to V-representation polytope
        Polytope vrep_polytope(nh_, "constrained_allowable_motion_polytope", base_link_, AHrep, bhrep);
        vrep_polytope.transformCartesian(geometry_information.geometry_jacobians.back().topRows(3), offset_position);

        if (show_polytope)
        {
            vrep_polytope.plot(offset_position, color_pts, color_line);
        }

        return vrep_polytope;
    }

    Polytope ConstrainedManipulability::getVelocityPolytope(const sensor_msgs::JointState &joint_states,
                                                            bool show_polytope,
                                                            std::vector<double> color_pts,
                                                            std::vector<double> color_line)
    {
        Eigen::MatrixXd AHrep;
        Eigen::VectorXd bhrep;

        Polytope poly = getVelocityPolytope(joint_states,
                                            AHrep,
                                            bhrep,
                                            show_polytope,
                                            color_pts, color_line);
        return poly;
    }

    Polytope ConstrainedManipulability::getVelocityPolytope(const sensor_msgs::JointState &joint_states,
                                                            Eigen::MatrixXd AHrep,
                                                            Eigen::VectorXd bhrep,
                                                            bool show_polytope,
                                                            std::vector<double> color_pts,
                                                            std::vector<double> color_line)
    {
        Eigen::Affine3d base_T_ee;
        Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
        KDL::JntArray kdl_joint_positions(ndof_);

        jointStatetoKDLJointArray(joint_states, kdl_joint_positions);
        getKDLKinematicInformation(kdl_joint_positions, base_T_ee, base_J_ee);

        // Define Hyperplanes
        AHrep.resize(2 * ndof_, ndof_);
        AHrep.topRows(ndof_) = ndof_identity_matrix_;                 // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

        // Define shifted distance from origin
        bhrep.resize(2 * ndof_, 1);
        for (int i = 0; i < ndof_; ++i)
        {
            bhrep(i) = qdotmax_[i];
            bhrep(i + ndof_) = -qdotmin_[i];
        }

        // Convert to V-representation polytope
        Polytope vrep_polytope(nh_, "velocity_polytope", base_link_, AHrep, bhrep);
        // Transform to Cartesian Space
        vrep_polytope.transformCartesian(base_J_ee.topRows(3), base_T_ee.translation());

        if (show_polytope)
        {
            vrep_polytope.plot(base_T_ee.translation(), color_pts, color_line);
        }

        // Return the calculated volume
        return vrep_polytope;
    }

    Polytope ConstrainedManipulability::getConstrainedVelocityPolytope(const sensor_msgs::JointState &joint_states,
                                                                       bool show_polytope,
                                                                       std::vector<double> color_pts,
                                                                       std::vector<double> color_line)
    {

        Eigen::MatrixXd AHrep;
        Eigen::VectorXd bhrep;

        Polytope poly = getConstrainedVelocityPolytope(joint_states,
                                                       AHrep, bhrep,
                                                       show_polytope,
                                                       color_pts,
                                                       color_line);
        return poly;
    }

    Polytope ConstrainedManipulability::getConstrainedVelocityPolytope(const sensor_msgs::JointState &joint_states,
                                                                       Eigen::MatrixXd &AHrep,
                                                                       Eigen::VectorXd &bhrep,
                                                                       bool show_polytope,
                                                                       std::vector<double> color_pts,
                                                                       std::vector<double> color_line)
    {
        GeometryInformation geometry_information;
        KDL::JntArray kdl_joint_positions(ndof_);

        jointStatetoKDLJointArray(joint_states, kdl_joint_positions);
        getCollisionModel(kdl_joint_positions, geometry_information);

        bool collision_free = getPolytopeHyperPlanes(kdl_joint_positions,
                                                     geometry_information,
                                                     AHrep,
                                                     bhrep, true);

        if (!collision_free)
        {
            return Polytope();
        }

        // Convert to V-representation polytope
        Polytope vrep_polytope(nh_, "constrained_velocity_polytope", base_link_, AHrep, bhrep);
        vrep_polytope.transformCartesian(geometry_information.geometry_jacobians.back().topRows(3),
                                         geometry_information.geometry_transforms.back().translation());

        if (show_polytope)
        {
            vrep_polytope.plot(geometry_information.geometry_transforms.back().translation(), color_pts, color_line);
        }

        return vrep_polytope;
    }

    bool ConstrainedManipulability::getPolytopeHyperPlanes(const KDL::JntArray &kdl_joint_positions,
                                                           GeometryInformation &geometry_information,
                                                           Eigen::MatrixXd &AHrep,
                                                           Eigen::VectorXd &bhrep,
                                                           bool velocity_polytope)
    {
        // Object ids
        std::vector<int> obj_ids;
        // Closest points
        std::vector<Eigen::Vector3d> p1w, p2w;
        // Min distance to object
        std::vector<double> obj_distances;
        // vector towards the object
        Eigen::Vector3d nt;
        // Vector of distance
        std::vector<double> distances;
        distances.clear();
        std::vector<Eigen::Matrix<double, 1, Eigen::Dynamic>> J_constraints;
        J_constraints.clear();

        // For all robot links
        for (int i = 0; i < geometry_information.shapes.size(); i++)
        {
            boost::mutex::scoped_lock lock(collision_world_mutex_);
            shapes::ShapeMsg current_shape;
            shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shape);

            if (current_shape.which() == 0)
            {
                fclInterface.checkDistanceObjectWorld(boost::get<shape_msgs::SolidPrimitive>(current_shape),
                                                      geometry_information.geometry_transforms[i],
                                                      obj_ids,
                                                      obj_distances,
                                                      p1w,
                                                      p2w);
            }
            else if (current_shape.which() == 1)
            {
                fclInterface.checkDistanceObjectWorld(boost::get<shape_msgs::Mesh>(current_shape),
                                                      geometry_information.geometry_transforms[i],
                                                      obj_ids,
                                                      obj_distances,
                                                      p1w,
                                                      p2w);
            }
            else
            {
                ROS_ERROR("Collision Geometry not support");
            }

            for (unsigned int j = 0; j < obj_distances.size(); j++)
            {

                Eigen::Matrix<double, 6, Eigen::Dynamic> w_J_out_p1;
                Eigen::Matrix<double, 1, Eigen::Dynamic> J_proj;
                J_proj.setZero();
                w_J_out_p1.setZero();

                if (obj_distances[j] < 0.0)
                {
                    // ROS_WARN ( " In collision" );
                    return false;
                }
                else if (obj_distances[j] < distance_threshold_)
                {
                    Eigen::Vector3d rdiff = p2w[j] - p1w[j];
                    if (obj_ids[j] == octomap_id_)
                    {
                        rdiff = p1w[j] - p2w[j]; // I really don't know why the octomap vector is backwards
                    }
                    nt = rdiff; // direction of obstacle
                    nt.normalize();

                    Eigen::Vector3d w_delta_p1_collision_origin = p1w[j] - geometry_information.geometry_transforms[i].translation();

                    screwTransform(geometry_information.geometry_jacobians[i],
                                   w_J_out_p1,
                                   w_delta_p1_collision_origin);
                    projectTranslationalJacobian(nt, w_J_out_p1, J_proj);
                    J_constraints.push_back(J_proj);

                    if (velocity_polytope)
                    {
                        distances.push_back(dangerfield_ * (obj_distances[j] * obj_distances[j]) - obj_distances[j]);
                    }
                    else
                    {
                        distances.push_back(obj_distances[j]);
                    }
                }
            }
        }

        // For velocity polytope there are ndof*2 less hyperplanes
        int offset(4);
        if (velocity_polytope)
        {
            offset = 2;
        }
        // Define Hyperplanes
        AHrep.resize(offset * ndof_ + J_constraints.size(),
                     ndof_);
        bhrep.resize(offset * ndof_ + distances.size(), 1);
        AHrep.setZero();
        bhrep.setZero();

        if (velocity_polytope)
        {                                                                 // If velocity , then the joint position constraints simply scale max velocity
            AHrep.topRows(ndof_) = ndof_identity_matrix_;                 // ndof_*ndof block at row  0 colum 0;ndof_
            AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

            for (int i = 0; i < ndof_; ++i)
            {

                double qmean = (qmax_[i] + qmin_[i]) / 2;
                double val_max = fmax(qmean, kdl_joint_positions(i)) - qmean;
                double val_min = fmin(qmean, kdl_joint_positions(i)) - qmean;
                double dmax = pow((((val_max) / ((qmax_[i] - qmean)))), 2);
                double dmin = pow((((val_min) / ((qmin_[i] - qmean)))), 2);

                // Make sure the value is witin joint limits and these limits are correctly defined.
                ROS_ASSERT(~std::isnan(dmax) && ~std::isnan(dmin) && ~std::isinf(dmax) && ~std::isinf(dmin));
                // Scale the maximum joint velocity based on joint position limits
                bhrep(i) = (1 - dmax) * qdotmax_[i];
                bhrep(i + ndof_) = (1 - dmin) * -qdotmin_[i];
            }
        }
        else
        {
            AHrep.topRows(ndof_) = ndof_identity_matrix_;                     // ndof_*ndof block at row  0 colum 0;ndof_
            AHrep.block(ndof_, 0, ndof_, ndof_) = ndof_identity_matrix_;      // ndof_*ndof block at row  ndof_ colum 0;ndof_
            AHrep.block(ndof_ * 2, 0, ndof_, ndof_) = ndof_identity_matrix_;  // ndof_*ndof block at row  ndof_*2 colum 0;ndof_
            AHrep.block(ndof_ * 3, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_*3 colum 0;ndof_ -ndof_identity_matrix_ for negative joint limits
            for (int i = 0; i < ndof_; ++i)
            {
                bhrep(i) = qmax_[i] - kdl_joint_positions(i);
                bhrep(i + ndof_) = kdl_joint_positions(i) - qmin_[i];
                bhrep(i + 2 * ndof_) = max_lin_limit_[i];
                bhrep(i + 3 * ndof_) = -min_lin_limit_[i];
            }
        }

        constrained_manipulability::ObjectDistances dist_arr;
        for (int var = 0; var < J_constraints.size(); ++var)
        {
            AHrep.row(offset * ndof_ + var) = J_constraints[var];
            bhrep(offset * ndof_ + var) = distances[var]; // Small tolerance to stop passing through
            dist_arr.distances.push_back(distances[var]);
        }

        dist_arr.stamp = ros::Time::now();
        obj_dist_pub_.publish(dist_arr);

        return true;
    }

    void ConstrainedManipulability::convertCollisionModel(const GeometryInformation &geometry_information,
                                                          std::vector<shapes::ShapeMsg> &current_shapes,
                                                          std::vector<geometry_msgs::Pose> &shapes_poses)
    {

        current_shapes.resize(geometry_information.geometry_transforms.size());
        shapes_poses.resize(geometry_information.geometry_transforms.size());

        for (int i = 0; i < geometry_information.geometry_transforms.size(); i++)
        {
            shapes::ShapeMsg current_shape;
            shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shapes[i]);
            tf::poseEigenToMsg(geometry_information.geometry_transforms[i], shapes_poses[i]);
        }
    }

    bool ConstrainedManipulability::getJacobianCallback(constrained_manipulability::GetJacobianMatrix::Request &req,
                                                        constrained_manipulability::GetJacobianMatrix::Response &res)
    {

        Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
        getJacobian(req.joint_states, base_J_ee);
        tf::matrixEigenToMsg(base_J_ee, res.jacobian);
        return true;
    }

    bool ConstrainedManipulability::getPolytopeConstraintsCallback(constrained_manipulability::GetPolytopeConstraints::Request &req,
                                                                   constrained_manipulability::GetPolytopeConstraints::Response &res)
    {
        res.polytope_hyperplanes.resize(req.sampled_joint_states.size());
        bool server_return = false;
        for (int var = 0; var < req.sampled_joint_states.size(); ++var)
        {
            Eigen::MatrixXd AHrep;
            Eigen::VectorXd bhrep;
            // Safe to assign, no pointer members
            Polytope poly;
            if (req.polytope_type == req.ALLOWABLE_MOTION_POLYTOPE)
            {
                poly = getAllowableMotionPolytope(req.sampled_joint_states[var], AHrep, bhrep, req.show_polytope);
            }
            else if (req.polytope_type == req.CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE)
            {
                poly = getConstrainedAllowableMotionPolytope(req.sampled_joint_states[var], AHrep, bhrep, req.show_polytope);
            }
            else if (req.polytope_type == req.CONSTRAINED_VELOCITY_POLYTOPE)
            {
                poly = getConstrainedVelocityPolytope(req.sampled_joint_states[var], AHrep, bhrep, req.show_polytope);
            }
            else if (req.polytope_type == req.VELOCITY_POLYTOPE)
            {
                poly = getVelocityPolytope(req.sampled_joint_states[var], AHrep, bhrep, req.show_polytope);
            }
            else
            {
                ROS_ERROR("Unknown Polytope type");
                return false;
            }

            tf::matrixEigenToMsg(AHrep, res.polytope_hyperplanes[var].A);
            res.polytope_hyperplanes[var].b = constrained_manipulability::eigenToVector(bhrep);
            res.polytope_hyperplanes[var].volume = poly.getVolume();
            server_return = true;
        }
        return server_return;
    }

    void ConstrainedManipulability::setLinearizationLimit(double linearization_limit, unsigned int joint)
    {
        max_lin_limit_[joint] = linearization_limit;
        min_lin_limit_[joint] = linearization_limit;
    }

    void ConstrainedManipulability::setLinearizationLimit(double linearization_limit)
    {
        std::fill(max_lin_limit_.begin(), max_lin_limit_.end(), linearization_limit);
        std::fill(min_lin_limit_.begin(), min_lin_limit_.end(), -linearization_limit);
    }

    double ConstrainedManipulability::getLinearizationLimit()
    {
        return max_lin_limit_[0];
    }

    bool ConstrainedManipulability::addCollisionObject(const shape_msgs::SolidPrimitive &s1,
                                                       const Eigen::Affine3d &wT1, unsigned int object_id)
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
        return fclInterface.addCollisionObject(s1, wT1, object_id);
    }

    bool ConstrainedManipulability::addCollisionObject(const shape_msgs::Mesh &s1,
                                                       const Eigen::Affine3d &wT1, unsigned int object_id)
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
        return fclInterface.addCollisionObject(s1, wT1, object_id);
    }

    bool ConstrainedManipulability::addCollisionObject(const octomap_msgs::Octomap &map,
                                                       const Eigen::Affine3d &wT1, unsigned int object_id)
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
        return fclInterface.addCollisionObject(map, wT1, object_id);
    }

    bool ConstrainedManipulability::addCollisionObject(robot_collision_checking::FCLObjectSet objects)
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
        return fclInterface.addCollisionObject(objects);
    }

    bool ConstrainedManipulability::removeCollisionObject(unsigned int object_id)
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
        return fclInterface.removeCollisionObject(object_id);
    }

    bool ConstrainedManipulability::displayObjects()
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
        return fclInterface.displayObjects(base_link_);
    }

    bool ConstrainedManipulability::displayCollisionModel(sensor_msgs::JointState const &joint_state)
    {

        KDL::JntArray kdl_joint_positions(ndof_);
        jointStatetoKDLJointArray(joint_state, kdl_joint_positions);

        GeometryInformation geometry_information;
        // Collision Link transforms
        getCollisionModel(kdl_joint_positions, geometry_information);
        for (int i = 0; i < geometry_information.geometry_transforms.size(); ++i)
        {
            visualization_msgs::Marker mk;
            shapes::constructMarkerFromShape(geometry_information.shapes[i].get(), mk, false);
            displayMarker(mk, geometry_information.geometry_transforms[i], base_link_, i, {0.1, 0.5, 0.2, 0.5});
        }

        return false;
    }

    bool ConstrainedManipulability::displayMarker(visualization_msgs::Marker mkr,
                                                  const Eigen::Affine3d &T,
                                                  std::string frame,
                                                  unsigned int obj_id,
                                                  const Eigen::Vector4d &color)
    {
        mkr.action = visualization_msgs::Marker::ADD;
        mkr.header.frame_id = frame;
        mkr.ns = "Objects";
        mkr.lifetime = ros::Duration(0.0);
        mkr.id = obj_id;
        mkr.color.r = color(0);
        mkr.color.g = color(1);
        mkr.color.b = color(2);
        mkr.color.a = color(3);

        Eigen::Quaterniond q(T.linear());
        mkr.pose.position.x = T(0, 3);
        mkr.pose.position.y = T(1, 3);
        mkr.pose.position.z = T(2, 3);
        mkr.pose.orientation.w = q.w();
        mkr.pose.orientation.x = q.x();
        mkr.pose.orientation.y = q.y();
        mkr.pose.orientation.z = q.z();
        mkr_pub_.publish(mkr);

        return true;
    }

    void ConstrainedManipulability::getCartPos(const sensor_msgs::JointState &joint_states,
                                               geometry_msgs::Pose &geo_pose)
    {

        KDL::JntArray kdl_joint_positions(ndof_);
        jointStatetoKDLJointArray(joint_states, kdl_joint_positions);

        KDL::Frame cartpos;
        kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos);
        Eigen::Affine3d T;
        tf::transformKDLToEigen(cartpos, T);
        tf::poseEigenToMsg(T, geo_pose);
    }

    void ConstrainedManipulability::getKDLKinematicInformation(const KDL::JntArray &kdl_joint_positions,
                                                               Eigen::Affine3d &T,
                                                               Eigen::Matrix<double, 6, Eigen::Dynamic> &Jac, int segment)
    {

        KDL::Frame cartpos;
        KDL::Jacobian base_J_link_origin;
        base_J_link_origin.resize(ndof_);
        if (segment != -1)
        {
            kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos, segment);
            kdl_dfk_solver_->JntToJac(kdl_joint_positions, base_J_link_origin, segment);
        }
        else
        {
            kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos);
            kdl_dfk_solver_->JntToJac(kdl_joint_positions, base_J_link_origin);
        }
        tf::transformKDLToEigen(cartpos, T);

        Jac = base_J_link_origin.data;
    }

    bool ConstrainedManipulability::checkCollision(const sensor_msgs::JointState &joint_states)
    {
        KDL::JntArray kdl_joint_positions(ndof_);
        jointStatetoKDLJointArray(joint_states, kdl_joint_positions);

        GeometryInformation geometry_information;
        getCollisionModel(kdl_joint_positions, geometry_information);

        std::vector<shapes::ShapeMsg> current_shapes;
        std::vector<geometry_msgs::Pose> shapes_poses;

        convertCollisionModel(geometry_information, current_shapes, shapes_poses);

        boost::mutex::scoped_lock lock(collision_world_mutex_);

        std::vector<geometry_msgs::PoseStamped> shapes_poses_stamped;
        for (int i = 0; i < shapes_poses.size(); i++)
        {
            geometry_msgs::PoseStamped shape_stamped;
            shape_stamped.header.stamp = joint_states.header.stamp;
            shape_stamped.header.frame_id = base_link_;
            shape_stamped.pose = shapes_poses[i];
            shapes_poses_stamped.push_back(shape_stamped);
        }

        // If octomap is available, then filter robot from octomap and add to collision world
        if (octo_filter_->addObjectToOctoFilter(current_shapes, shapes_poses_stamped))
        {
            // Get the octomap properties, including its pose in the robot base frame
            octomap_msgs::Octomap octomap;
            geometry_msgs::TransformStamped octomap_wrt_base;
            octo_filter_->getOctomapProperties(base_link_, octomap, octomap_wrt_base);
            Eigen::Affine3d octomap_pose_wrt_base;
            tf::transformMsgToEigen(octomap_wrt_base.transform, octomap_pose_wrt_base);

            // Remove the old octomap from the world
            fclInterface.removeCollisionObject(octomap_id_);
            // Update with the new
            fclInterface.addCollisionObject(octomap, octomap_pose_wrt_base, octomap_id_);
        }

        for (int i = 0; i < geometry_information.geometry_transforms.size(); i++)
        {
            shapes::ShapeMsg current_shape;
            shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shape);

            if (current_shape.which() == 0)
            {
                if (fclInterface.checkCollisionObjectWorld(boost::get<shape_msgs::SolidPrimitive>(current_shape),
                                                           geometry_information.geometry_transforms[i]))
                {
                    return true;
                }
            }
            else if (current_shape.which() == 1)
            {
                if (fclInterface.checkCollisionObjectWorld(boost::get<shape_msgs::Mesh>(current_shape),
                                                           geometry_information.geometry_transforms[i]))
                {
                    return true;
                }
            }
            else
            {
                ROS_ERROR("Collision Geometry not support");
            }
        }

        return false;
    }

    void ConstrainedManipulability::getTransform(const sensor_msgs::JointState &joint_states, Eigen::Affine3d &T)
    {
        KDL::JntArray kdl_joint_positions(ndof_);
        Eigen::Matrix<double, 6, Eigen::Dynamic> Jac;
        jointStatetoKDLJointArray(joint_states, kdl_joint_positions);
        getKDLKinematicInformation(kdl_joint_positions, T, Jac);
    }

    void ConstrainedManipulability::getJacobian(const sensor_msgs::JointState &joint_states, Eigen::Matrix<double, 6, Eigen::Dynamic> &Jac)
    {
        KDL::JntArray kdl_joint_positions(ndof_);
        Eigen::Affine3d base_T_ee;
        jointStatetoKDLJointArray(joint_states, kdl_joint_positions);
        getKDLKinematicInformation(kdl_joint_positions, base_T_ee, Jac);
    }

    std::unique_ptr<shapes::Shape> ConstrainedManipulability::constructShape(const urdf::Geometry *geom)
    {
        ROS_ASSERT(geom);

        std::unique_ptr<shapes::Shape> result = NULL;
        switch (geom->type)
        {
        case urdf::Geometry::SPHERE:
        {
            result = std::unique_ptr<shapes::Shape>(new shapes::Sphere(dynamic_cast<const urdf::Sphere *>(geom)->radius));
            break;
        }
        case urdf::Geometry::BOX:
        {
            urdf::Vector3 dim = dynamic_cast<const urdf::Box *>(geom)->dim;
            result = std::unique_ptr<shapes::Shape>(new shapes::Box(dim.x, dim.y, dim.z));
            break;
        }
        case urdf::Geometry::CYLINDER:
        {
            result = std::unique_ptr<shapes::Shape>(new shapes::Cylinder(dynamic_cast<const urdf::Cylinder *>(geom)->radius,
                                                                         dynamic_cast<const urdf::Cylinder *>(geom)->length));
            break;
        }
        case urdf::Geometry::MESH:
        {
            const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh *>(geom);
            if (!mesh->filename.empty())
            {
                Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
                result = std::unique_ptr<shapes::Shape>(shapes::createMeshFromResource(mesh->filename, scale));
            }
            else
            {
                ROS_WARN("Empty mesh filename");
            }
            break;
        }
        default:
        {
            ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
            break;
        }
        }
        return (result);
    }

    void ConstrainedManipulability::getCollisionModel(const KDL::JntArray &kdl_joint_positions,
                                                      GeometryInformation &geometry_information)
    {

        geometry_information.clear();
        Eigen::Affine3d link_origin_T_collision_origin, base_T_link_origin, base_T_collision_origin;

        // Calculates the segment's collision geomtery
        // The transform to the origin of the collision geometry
        // The Jacobian matrix at the origin of the collision geometry
        for (int i = 0; i < chain_.getNrOfSegments(); ++i)
        {
            KDL::Segment seg = chain_.getSegment(i); // Get current segment

            // Get Collision Geometry
            std::unique_ptr<shapes::Shape> shape = constructShape(model_.links_.at(seg.getName())->collision->geometry.get());

            // Get Collision Origin
            Eigen::Vector3d origin_Trans_collision(model_.links_.at(seg.getName())->collision->origin.position.x,
                                                   model_.links_.at(seg.getName())->collision->origin.position.y,
                                                   model_.links_.at(seg.getName())->collision->origin.position.z);
            Eigen::Quaterniond origin_Quat_collision(
                model_.links_.at(seg.getName())->collision->origin.rotation.w,
                model_.links_.at(seg.getName())->collision->origin.rotation.x,
                model_.links_.at(seg.getName())->collision->origin.rotation.y,
                model_.links_.at(seg.getName())->collision->origin.rotation.z);

            link_origin_T_collision_origin.translation() = origin_Trans_collision;
            link_origin_T_collision_origin.linear() = origin_Quat_collision.toRotationMatrix();

            // Finds cartesian pose w.r.t to base frame
            Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_collision_origin, base_J_link_origin;
            getKDLKinematicInformation(kdl_joint_positions, base_T_link_origin, base_J_link_origin, i + 1);
            base_T_collision_origin = base_T_link_origin * link_origin_T_collision_origin;
            Eigen::Vector3d base_L_link_collision = (base_T_link_origin.linear() * link_origin_T_collision_origin.translation());
            // Screw transform to collision origin
            screwTransform(base_J_link_origin, base_J_collision_origin, base_L_link_collision);

            // Push back solutions
            geometry_information.shapes.push_back(std::move(shape));
            geometry_information.geometry_transforms.push_back(base_T_collision_origin);
            geometry_information.geometry_jacobians.push_back(base_J_collision_origin);
        }
    }

    bool ConstrainedManipulability::projectTranslationalJacobian(Eigen::Vector3d nT,
                                                                 const Eigen::Matrix<double, 6, Eigen::Dynamic> &J0N_in,
                                                                 Eigen::Matrix<double, 1, Eigen::Dynamic> &J0N_out)
    {

        int n = J0N_in.cols();
        ROS_ASSERT(J0N_in.rows() > 3);
        J0N_out.setZero();
        J0N_out = nT.transpose() * J0N_in.topLeftCorner(3, n);
        return true;
    }

    bool ConstrainedManipulability::screwTransform(const Eigen::Matrix<double, 6, Eigen::Dynamic> &J0N_in,
                                                   Eigen::Matrix<double, 6, Eigen::Dynamic> &J0E_out,
                                                   const Eigen::Vector3d &L)
    {
        J0E_out.setZero();
        Eigen::Matrix<double, 3, 3> Lhat;
        Lhat.setZero();
        skew(L, Lhat);
        Eigen::Matrix<double, 6, 6> screwL;
        screwL.setZero();
        screwL.topLeftCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
        screwL.bottomRightCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
        screwL.bottomLeftCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Zero();
        screwL.topRightCorner(3, 3) = -Lhat;
        J0E_out = screwL * J0N_in;
        return true;
    }

    void ConstrainedManipulability::jointStatetoKDLJointArray(const sensor_msgs::JointState &joint_states,
                                                              KDL::JntArray &kdl_joint_positions)
    {
        unsigned int jnt(0);
        for (int i = 0; i < chain_.getNrOfSegments(); ++i)
        {

            KDL::Segment seg = chain_.getSegment(i);
            KDL::Joint kdl_joint = seg.getJoint();
            for (int j = 0; j < joint_states.name.size(); ++j)
            {
                if (kdl_joint.getName() == joint_states.name[j])
                {
                    kdl_joint_positions(jnt) = joint_states.position[j];
                    jnt++;
                }
            }
        }
    }
}