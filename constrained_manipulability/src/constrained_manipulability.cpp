#include <cassert>
#include <chrono>

#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "constrained_manipulability_interfaces/msg/matrix.hpp"

#include "constrained_manipulability/constrained_manipulability.hpp"

namespace constrained_manipulability
{
ConstrainedManipulability::ConstrainedManipulability() : Node("constrained_manipulability")
{
    // Populate private properties
    base_link_ = this->declare_parameter<std::string>("root", "base_link");
    tip_ = this->declare_parameter<std::string>("tip", "ee");

    distance_threshold_ = this->declare_parameter<double>("distance_threshold", 0.3);
    dangerfield_ = this->declare_parameter<double>("dangerfield", 10.0);

    double lin_limit = this->declare_parameter<double>("linearization_limit", 0.1);

    // Create collision world
    collision_world_ = std::make_shared<robot_collision_checking::FCLInterfaceCollisionWorld>(base_link_);

    // Instantiate ROS services and publishers
    jacobian_server_ = this->create_service<constrained_manipulability_interfaces::srv::GetJacobianMatrix>(
        "get_jacobian_matrix",  std::bind(&ConstrainedManipulability::getJacobianCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));
    polytope_server_ = this->create_service<constrained_manipulability_interfaces::srv::GetPolytopeConstraints>(
        "get_polytope_constraints",  std::bind(&ConstrainedManipulability::getPolytopeConstraintsCallback, this,
                                               std::placeholders::_1, std::placeholders::_2));

    mkr_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 1);
    obj_dist_pub_ = this->create_publisher<constrained_manipulability_interfaces::msg::ObjectDistances>("constrained_manipulability/obj_distances", 1);
    poly_mesh_pub_ = this->create_publisher<constrained_manipulability_interfaces::msg::PolytopeMesh>("constrained_manipulability/polytope_mesh", 1);
    poly_vol_pub_ = this->create_publisher<constrained_manipulability_interfaces::msg::PolytopeVolume>("constrained_manipulability/polytope_volume", 1);
    obj_filt_pub_ = this->create_publisher<octomap_filter_interfaces::msg::FilterMesh>("filter_mesh", 1);

    // Create parameter client
    auto params_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");
    while (!params_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Robot state description service not available, waiting...");
    }

    // Robot kinematics model creation
    std::string urdf_string;
    auto parameters = params_client->get_parameters({ "robot_description" });
    for (auto& parameter : parameters)
    {
        if (parameter.get_name() == "robot_description")
        {
            urdf_string = parameter.value_to_string();
            break;
        }
    }

    // Initialize URDF model and KDL tree
    model_->initString(urdf_string);
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(*model_, tree))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
    }

    tree.getChain(base_link_, tip_, chain_);
    ndof_ = chain_.getNrOfJoints();
    int nseg = chain_.getNrOfSegments();
    RCLCPP_INFO(this->get_logger(), "Loading tree from parameter %s with kinematic chain from %s to %s", urdf_string.c_str(), base_link_.c_str(), tip_.c_str());
    RCLCPP_INFO(this->get_logger(), "Number of tree segments: %d", tree.getNrOfSegments());
    RCLCPP_INFO(this->get_logger(), "Number of tree joints: %d", tree.getNrOfJoints());
    RCLCPP_INFO(this->get_logger(), "Number of chain joints: %d", ndof_);
    RCLCPP_INFO(this->get_logger(), "Number of chain segments: %d", nseg);

    if (ndof_ < 1)
    {
        RCLCPP_FATAL(this->get_logger(), "Robot has 0 joints, check if root and/or tip name is incorrect!");
        rclcpp::shutdown();
    }

    qmax_.resize(ndof_);
    qmin_.resize(ndof_);
    qdotmax_.resize(ndof_);
    qdotmin_.resize(ndof_);
    max_lin_limit_.resize(ndof_);
    min_lin_limit_.resize(ndof_);
    setLinearizationLimit(lin_limit);

    kdl_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    kdl_dfk_solver_.reset(new KDL::ChainJntToJacSolver(chain_));

    int mvable_jnt(0);
    std::vector<std::string> joint_names(ndof_);
    for (int i = 0; i < nseg; ++i)
    {
        KDL::Segment seg = chain_.getSegment(i);
        KDL::Joint kdl_joint = seg.getJoint();
        urdf::JointConstSharedPtr urdf_joint = model_->getJoint(kdl_joint.getName());

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
                qmax_[mvable_jnt] = model_->joints_.at(kdl_joint.getName())->limits->upper;
                qmin_[mvable_jnt] = model_->joints_.at(kdl_joint.getName())->limits->lower;
            }

            qdotmax_[mvable_jnt] = model_->joints_.at(kdl_joint.getName())->limits->velocity;
            qdotmin_[mvable_jnt] = -model_->joints_.at(kdl_joint.getName())->limits->velocity;
            joint_names[mvable_jnt] = kdl_joint.getName();
            mvable_jnt++;
        }
    }

    // TODO: Use ROS 2 way of setting parameters
    // nh.setParam("constrained_manipulability/active_dof", joint_names);

    // client = this->create_client<rcl_interfaces::srv::SetParametersAtomically>(serviceName); // E.g.: serviceName = "constrained_manipulability/active_dof"

    // parameter.name = parameter_name;  // E.g.: parameter_name = "background_b"
    // parameter.value.type = 1          //  bool = 1,    int = 2,        float = 3,     string = 4
    // parameter.value.bool_value = true // .bool_value, .integer_value, .double_value, .string_value

    // request->parameters.push_back(parameter);

    // while (!client->wait_for_service(1s)) {
    //     if (!rclcpp::ok()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    //     return;
    //     }
    //     RCLCPP_INFO_STREAM(this->get_logger(), "service " << serviceName <<" not available, waiting again..."); 
    // }
    // auto result = client->async_send_request(request);

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

    printVector(qmax_, "qmax_");
    printVector(qmin_, "qmin_");
    printVector(qdotmax_, "qdotmax_");
    printVector(qdotmin_, "qdotmin_");

    RCLCPP_INFO(this->get_logger(), "Initialized constrained_manipulability");
}

void ConstrainedManipulability::getJacobianCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetJacobianMatrix::Request> req,
                                                    std::shared_ptr<constrained_manipulability_interfaces::srv::GetJacobianMatrix::Response> res)
{

    Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
    getJacobian(req->joint_states, base_J_ee);

    constrained_manipulability_interfaces::msg::Matrix mat;
    mat.rows = base_J_ee.rows();
    mat.columns = base_J_ee.cols();
    int ii = 0;
    for (uint i = 0; i < base_J_ee.rows(); ++i)
    {
        for (int j = 0; j < base_J_ee.cols(); ++j)
        {
            mat.data[ii++] = base_J_ee.coeff(i, j);
        }
    }

    res->jacobian = mat;
}

void ConstrainedManipulability::getPolytopeConstraintsCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetPolytopeConstraints::Request> req,
                                                               std::shared_ptr<constrained_manipulability_interfaces::srv::GetPolytopeConstraints::Response> res)
{
    int num_states = req->sampled_joint_states.size();
    res->polytope_hyperplanes.resize(num_states);
    for (int var = 0; var < num_states; ++var)
    {
        Eigen::MatrixXd AHrep;
        Eigen::VectorXd bhrep;
        // Safe to assign, no pointer members
        // Polytope poly;
        // if (req->polytope_type == req->ALLOWABLE_MOTION_POLYTOPE)
        // {
        //     poly = getAllowableMotionPolytope(req->sampled_joint_states[var], AHrep, bhrep, req->show_polytope);
        // }
        // else if (req->polytope_type == req->CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE)
        // {
        //     poly = getConstrainedAllowableMotionPolytope(req->sampled_joint_states[var], AHrep, bhrep, req->show_polytope);
        // }
        // else if (req->polytope_type == req->CONSTRAINED_VELOCITY_POLYTOPE)
        // {
        //     poly = getConstrainedVelocityPolytope(req->sampled_joint_states[var], AHrep, bhrep, req->show_polytope);
        // }
        // else if (req->polytope_type == req->VELOCITY_POLYTOPE)
        // {
        //     poly = getVelocityPolytope(req->sampled_joint_states[var], AHrep, bhrep, req->show_polytope);
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Unknown Polytope type");
        // }

        // res->polytope_hyperplanes[var].A = tf2::toMsg(AHrep);
        // res->polytope_hyperplanes[var].b = constrained_manipulability::eigenToVector(bhrep);
        // res->polytope_hyperplanes[var].volume = poly.getVolume();
    }
}

// Polytope ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
//                                                                 bool show_polytope,
//                                                                 std::vector<double> color_pts,
//                                                                 std::vector<double> color_line)
// {
//     Eigen::MatrixXd AHrep;
//     Eigen::VectorXd bhrep;
//     Eigen::Vector3d offset_position;

//     Polytope poly = getAllowableMotionPolytope(joint_states,
//                                                 AHrep,
//                                                 bhrep,
//                                                 offset_position,
//                                                 show_polytope,
//                                                 color_pts,
//                                                 color_line);
//     return poly;
// }

// Polytope ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
//                                                                 Eigen::MatrixXd& AHrep,
//                                                                 Eigen::VectorXd& bhrep,
//                                                                 bool show_polytope,
//                                                                 std::vector<double> color_pts,
//                                                                 std::vector<double> color_line)
// {
//     Eigen::Vector3d offset_position;

//     Polytope poly = getAllowableMotionPolytope(joint_states,
//                                                 AHrep,
//                                                 bhrep,
//                                                 offset_position,
//                                                 show_polytope,
//                                                 color_pts,
//                                                 color_line);
//     return poly;
// }

// Polytope ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
//                                                                 Eigen::MatrixXd& AHrep,
//                                                                 Eigen::VectorXd& bhrep,
//                                                                 Eigen::Vector3d& offset_position,
//                                                                 bool show_polytope,
//                                                                 std::vector<double> color_pts,
//                                                                 std::vector<double> color_line)
// {
//     Eigen::Affine3d base_T_ee;
//     Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
//     KDL::JntArray kdl_joint_positions(ndof_);

//     jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);
//     getKDLKinematicInformation(kdl_joint_positions, base_T_ee, base_J_ee);

//     // Define Hyperplanes
//     AHrep.resize(2 * ndof_, ndof_);
//     AHrep.topRows(ndof_) = ndof_identity_matrix_;                 // ndof_*ndof block at row  0 colum 0;ndof_
//     AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

//     // Define shifted distance from origin
//     bhrep.resize(2 * ndof_, 1);
//     for (int i = 0; i < ndof_; ++i)
//     {
//         bhrep(i) = max_lin_limit_[i];
//         bhrep(i + ndof_) = -min_lin_limit_[i];
//     }

//     // Define offset position to end effector
//     offset_position = base_T_ee.translation();

//     // Convert to V-representation polytope
//     Polytope vrep_polytope("allowable_motion_polytope", AHrep, bhrep);
//     // Transform to Cartesian Space
//     vrep_polytope.transformCartesian(base_J_ee.topRows(3), offset_position);

//     if (show_polytope)
//     {
//         plotPolytope(vrep_polytope, offset_position, color_pts, color_line);
//     }

//     // Publish polytope volume
//     constrained_manipulability::PolytopeVolume polytope_volume;
//     polytope_volume.name = vrep_polytope.getName();
//     polytope_volume.volume = vrep_polytope.getVolume();
//     poly_vol_pub_.publish(polytope_volume);

//     // Return the calculated polytope
//     return vrep_polytope;
// }

// Polytope ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
//                                                                             bool show_polytope,
//                                                                             std::vector<double> color_pts,
//                                                                             std::vector<double> color_line)
// {
//     Eigen::MatrixXd AHrep;
//     Eigen::VectorXd bhrep;
//     Eigen::Vector3d offset_position;

//     Polytope poly = getConstrainedAllowableMotionPolytope(joint_states,
//                                                             AHrep,
//                                                             bhrep,
//                                                             offset_position,
//                                                             show_polytope,
//                                                             color_pts,
//                                                             color_line);

//     return poly;
// }

// Polytope ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
//                                                                             Eigen::MatrixXd& AHrep,
//                                                                             Eigen::VectorXd& bhrep,
//                                                                             bool show_polytope,
//                                                                             std::vector<double> color_pts,
//                                                                             std::vector<double> color_line)
// {
//     Eigen::Vector3d offset_position;

//     Polytope poly = getConstrainedAllowableMotionPolytope(joint_states,
//                                                             AHrep,
//                                                             bhrep,
//                                                             offset_position,
//                                                             show_polytope,
//                                                             color_pts,
//                                                             color_line);
//     return poly;
// }

// Polytope ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
//                                                                             Eigen::MatrixXd& AHrep,
//                                                                             Eigen::VectorXd& bhrep,
//                                                                             Eigen::Vector3d& offset_position,
//                                                                             bool show_polytope,
//                                                                             std::vector<double> color_pts,
//                                                                             std::vector<double> color_line)
// {
//     GeometryInformation geometry_information;
//     KDL::JntArray kdl_joint_positions(ndof_);

//     jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);
//     getCollisionModel(kdl_joint_positions, geometry_information);

//     bool collision_free = getPolytopeHyperPlanes(kdl_joint_positions,
//                                                     geometry_information,
//                                                     AHrep,
//                                                     bhrep);

//     if (!collision_free)
//     {
//         return Polytope();
//     }

//     // Define offset position based on geometrical collision world for robot body
//     offset_position = geometry_information.geometry_transforms.back().translation();

//     // Convert to V-representation polytope
//     Polytope vrep_polytope("constrained_allowable_motion_polytope", AHrep, bhrep);
//     // Transform to Cartesian Space
//     vrep_polytope.transformCartesian(geometry_information.geometry_jacobians.back().topRows(3), offset_position);

//     if (show_polytope)
//     {
//         plotPolytope(vrep_polytope, offset_position, color_pts, color_line);
//     }

//     // Publish polytope volume
//     constrained_manipulability::PolytopeVolume polytope_volume;
//     polytope_volume.name = vrep_polytope.getName();
//     polytope_volume.volume = vrep_polytope.getVolume();
//     poly_vol_pub_.publish(polytope_volume);

//     // Return the calculated polytope
//     return vrep_polytope;
// }

// Polytope ConstrainedManipulability::getVelocityPolytope(const sensor_msgs::JointState& joint_states,
//                                                         bool show_polytope,
//                                                         std::vector<double> color_pts,
//                                                         std::vector<double> color_line)
// {
//     Eigen::MatrixXd AHrep;
//     Eigen::VectorXd bhrep;

//     Polytope poly = getVelocityPolytope(joint_states,
//                                         AHrep,
//                                         bhrep,
//                                         show_polytope,
//                                         color_pts, color_line);
//     return poly;
// }

// Polytope ConstrainedManipulability::getVelocityPolytope(const sensor_msgs::JointState& joint_states,
//                                                         Eigen::MatrixXd AHrep,
//                                                         Eigen::VectorXd bhrep,
//                                                         bool show_polytope,
//                                                         std::vector<double> color_pts,
//                                                         std::vector<double> color_line)
// {
//     Eigen::Affine3d base_T_ee;
//     Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
//     KDL::JntArray kdl_joint_positions(ndof_);

//     jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);
//     getKDLKinematicInformation(kdl_joint_positions, base_T_ee, base_J_ee);

//     // Define Hyperplanes
//     AHrep.resize(2 * ndof_, ndof_);
//     AHrep.topRows(ndof_) = ndof_identity_matrix_;                 // ndof_*ndof block at row  0 colum 0;ndof_
//     AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

//     // Define shifted distance from origin
//     bhrep.resize(2 * ndof_, 1);
//     for (int i = 0; i < ndof_; ++i)
//     {
//         bhrep(i) = qdotmax_[i];
//         bhrep(i + ndof_) = -qdotmin_[i];
//     }

//     // Convert to V-representation polytope
//     Polytope vrep_polytope("velocity_polytope", AHrep, bhrep);
//     // Transform to Cartesian Space
//     Eigen::Vector3d offset_position = base_T_ee.translation();
//     vrep_polytope.transformCartesian(base_J_ee.topRows(3), offset_position);

//     if (show_polytope)
//     {
//         plotPolytope(vrep_polytope, offset_position, color_pts, color_line);
//     }

//     // Publish polytope volume
//     constrained_manipulability::PolytopeVolume polytope_volume;
//     polytope_volume.name = vrep_polytope.getName();
//     polytope_volume.volume = vrep_polytope.getVolume();
//     poly_vol_pub_.publish(polytope_volume);

//     // Return the calculated polytope
//     return vrep_polytope;
// }

// Polytope ConstrainedManipulability::getConstrainedVelocityPolytope(const sensor_msgs::JointState& joint_states,
//                                                                     bool show_polytope,
//                                                                     std::vector<double> color_pts,
//                                                                     std::vector<double> color_line)
// {

//     Eigen::MatrixXd AHrep;
//     Eigen::VectorXd bhrep;

//     Polytope poly = getConstrainedVelocityPolytope(joint_states,
//                                                     AHrep, bhrep,
//                                                     show_polytope,
//                                                     color_pts,
//                                                     color_line);
//     return poly;
// }

// Polytope ConstrainedManipulability::getConstrainedVelocityPolytope(const sensor_msgs::JointState& joint_states,
//                                                                     Eigen::MatrixXd& AHrep,
//                                                                     Eigen::VectorXd& bhrep,
//                                                                     bool show_polytope,
//                                                                     std::vector<double> color_pts,
//                                                                     std::vector<double> color_line)
// {
//     GeometryInformation geometry_information;
//     KDL::JntArray kdl_joint_positions(ndof_);

//     jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);
//     getCollisionModel(kdl_joint_positions, geometry_information);

//     bool collision_free = getPolytopeHyperPlanes(kdl_joint_positions,
//                                                     geometry_information,
//                                                     AHrep,
//                                                     bhrep, true);

//     if (!collision_free)
//     {
//         return Polytope();
//     }

//     // Convert to V-representation polytope
//     Polytope vrep_polytope("constrained_velocity_polytope", AHrep, bhrep);
//     // Transform to Cartesian Space
//     Eigen::Vector3d offset_position = geometry_information.geometry_transforms.back().translation();
//     vrep_polytope.transformCartesian(geometry_information.geometry_jacobians.back().topRows(3), offset_position);

//     if (show_polytope)
//     {
//         plotPolytope(vrep_polytope, offset_position, color_pts, color_line);
//     }

//     // Publish polytope volume
//     constrained_manipulability::PolytopeVolume polytope_volume;
//     polytope_volume.name = vrep_polytope.getName();
//     polytope_volume.volume = vrep_polytope.getVolume();
//     poly_vol_pub_.publish(polytope_volume);

//     // Return the calculated polytope
//     return vrep_polytope;
// }

// bool ConstrainedManipulability::plotPolytope(const Polytope& poly,
//                                                 const Eigen::Vector3d& offset_position,
//                                                 std::vector<double> color_pts,
//                                                 std::vector<double> color_line) const
// {
//     std::vector<geometry_msgs::Point> points;
//     constrained_manipulability::PolytopeMesh poly_mesh;
//     bool success = poly.getPolytopeMesh(offset_position, points, poly_mesh);

//     if (success)
//     {
//         visualization_msgs::Marker mkr;
//         mkr.ns = poly_mesh.name;
//         mkr.action = visualization_msgs::Marker::ADD;
//         mkr.type = visualization_msgs::Marker::TRIANGLE_LIST;
//         mkr.header.frame_id = base_link_;
//         mkr.id = 2;
//         mkr.lifetime = ros::Duration(0.0);
//         mkr.color.r = color_line[0];
//         mkr.color.g = color_line[1];
//         mkr.color.b = color_line[2];
//         mkr.color.a = color_line[3]; // fmax(auto_alpha,0.1);
//         mkr.scale.x = 1.0;
//         mkr.scale.y = 1.0;
//         mkr.scale.z = 1.0;
//         mkr.points = points;

//         mkr_pub_.publish(mkr);

//         poly_mesh.color = mkr.color;
//         poly_mesh_pub_.publish(poly_mesh);

//         mkr.type = visualization_msgs::Marker::SPHERE_LIST;
//         mkr.header.frame_id = base_link_;
//         mkr.id = 1;
//         mkr.lifetime = ros::Duration(0.0);
//         mkr.color.r = color_pts[0];
//         mkr.color.g = color_pts[1];
//         mkr.color.b = color_pts[2];
//         mkr.color.a = color_pts[3];
//         mkr.scale.x = 0.005;
//         mkr.scale.y = 0.005;
//         mkr.scale.z = 0.005;
//         mkr.points = points;

//         mkr_pub_.publish(mkr);
//     }

//     return success;
// }

// bool ConstrainedManipulability::getPolytopeHyperPlanes(const KDL::JntArray& kdl_joint_positions,
//                                                         GeometryInformation& geometry_information,
//                                                         Eigen::MatrixXd& AHrep,
//                                                         Eigen::VectorXd& bhrep,
//                                                         bool velocity_polytope)
// {
//     // Object ids
//     std::vector<int> obj_ids;
//     // Closest points
//     std::vector<Eigen::Vector3d> p1w, p2w;
//     // Min distance to object
//     std::vector<double> obj_distances;
//     // vector towards the object
//     Eigen::Vector3d nt;
//     // Vector of distance
//     std::vector<double> distances;
//     distances.clear();
//     std::vector<Eigen::Matrix<double, 1, Eigen::Dynamic>> J_constraints;
//     J_constraints.clear();

//     // For all robot links
//     for (int i = 0; i < geometry_information.shapes.size(); i++)
//     {
//         boost::mutex::scoped_lock lock(collision_world_mutex_);
//         shapes::ShapeMsg current_shape;
//         shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shape);

//         if (current_shape.which() == 0)
//         {
//             fclInterface.checkDistanceObjectWorld(boost::get<shape_msgs::SolidPrimitive>(current_shape),
//                                                     geometry_information.geometry_transforms[i],
//                                                     obj_ids,
//                                                     obj_distances,
//                                                     p1w,
//                                                     p2w);
//         }
//         else if (current_shape.which() == 1)
//         {
//             fclInterface.checkDistanceObjectWorld(boost::get<shape_msgs::Mesh>(current_shape),
//                                                     geometry_information.geometry_transforms[i],
//                                                     obj_ids,
//                                                     obj_distances,
//                                                     p1w,
//                                                     p2w);
//         }
//         else
//         {
//             ROS_ERROR("Collision Geometry not support");
//         }

//         for (unsigned int j = 0; j < obj_distances.size(); j++)
//         {

//             Eigen::Matrix<double, 6, Eigen::Dynamic> w_J_out_p1;
//             Eigen::Matrix<double, 1, Eigen::Dynamic> J_proj;
//             J_proj.setZero();
//             w_J_out_p1.setZero();

//             if (obj_distances[j] < 0.0)
//             {
//                 // ROS_WARN ( " In collision" );
//                 return false;
//             }
//             else if (obj_distances[j] < distance_threshold_)
//             {
//                 Eigen::Vector3d rdiff = p2w[j] - p1w[j];
//                 if (obj_ids[j] == octomap_id_)
//                 {
//                     rdiff = p1w[j] - p2w[j]; // I really don't know why the octomap vector is backwards
//                 }
//                 nt = rdiff; // direction of obstacle
//                 nt.normalize();

//                 Eigen::Vector3d w_delta_p1_collision_origin = p1w[j] - geometry_information.geometry_transforms[i].translation();

//                 screwTransform(geometry_information.geometry_jacobians[i],
//                                 w_delta_p1_collision_origin,
//                                 w_J_out_p1);
//                 projectTranslationalJacobian(nt, w_J_out_p1, J_proj);
//                 J_constraints.push_back(J_proj);

//                 if (velocity_polytope)
//                 {
//                     distances.push_back(dangerfield_ * (obj_distances[j] * obj_distances[j]) - obj_distances[j]);
//                 }
//                 else
//                 {
//                     distances.push_back(obj_distances[j]);
//                 }
//             }
//         }
//     }

//     // For velocity polytope there are ndof*2 less hyperplanes
//     int offset(4);
//     if (velocity_polytope)
//     {
//         offset = 2;
//     }
//     // Define Hyperplanes
//     AHrep.resize(offset * ndof_ + J_constraints.size(),
//                     ndof_);
//     bhrep.resize(offset * ndof_ + distances.size(), 1);
//     AHrep.setZero();
//     bhrep.setZero();

//     if (velocity_polytope)
//     {                                                                 // If velocity , then the joint position constraints simply scale max velocity
//         AHrep.topRows(ndof_) = ndof_identity_matrix_;                 // ndof_*ndof block at row  0 colum 0;ndof_
//         AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

//         for (int i = 0; i < ndof_; ++i)
//         {

//             double qmean = (qmax_[i] + qmin_[i]) / 2;
//             double val_max = fmax(qmean, kdl_joint_positions(i)) - qmean;
//             double val_min = fmin(qmean, kdl_joint_positions(i)) - qmean;
//             double dmax = pow((((val_max) / ((qmax_[i] - qmean)))), 2);
//             double dmin = pow((((val_min) / ((qmin_[i] - qmean)))), 2);

//             // Make sure the value is witin joint limits and these limits are correctly defined.
//             assert(~std::isnan(dmax) && ~std::isnan(dmin) && ~std::isinf(dmax) && ~std::isinf(dmin));
//             // Scale the maximum joint velocity based on joint position limits
//             bhrep(i) = (1 - dmax) * qdotmax_[i];
//             bhrep(i + ndof_) = (1 - dmin) * -qdotmin_[i];
//         }
//     }
//     else
//     {
//         AHrep.topRows(ndof_) = ndof_identity_matrix_;                     // ndof_*ndof block at row  0 colum 0;ndof_
//         AHrep.block(ndof_, 0, ndof_, ndof_) = ndof_identity_matrix_;      // ndof_*ndof block at row  ndof_ colum 0;ndof_
//         AHrep.block(ndof_ * 2, 0, ndof_, ndof_) = ndof_identity_matrix_;  // ndof_*ndof block at row  ndof_*2 colum 0;ndof_
//         AHrep.block(ndof_ * 3, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_*3 colum 0;ndof_ -ndof_identity_matrix_ for negative joint limits
//         for (int i = 0; i < ndof_; ++i)
//         {
//             bhrep(i) = qmax_[i] - kdl_joint_positions(i);
//             bhrep(i + ndof_) = kdl_joint_positions(i) - qmin_[i];
//             bhrep(i + 2 * ndof_) = max_lin_limit_[i];
//             bhrep(i + 3 * ndof_) = -min_lin_limit_[i];
//         }
//     }

//     constrained_manipulability::ObjectDistances dist_arr;
//     for (int var = 0; var < J_constraints.size(); ++var)
//     {
//         AHrep.row(offset * ndof_ + var) = J_constraints[var];
//         bhrep(offset * ndof_ + var) = distances[var]; // Small tolerance to stop passing through
//         dist_arr.distances.push_back(distances[var]);
//     }

//     dist_arr.stamp = ros::Time::now();
//     obj_dist_pub_.publish(dist_arr);

//     return true;
// }

// bool ConstrainedManipulability::addCollisionObject(const shape_msgs::SolidPrimitive& s1,
//                                                     const Eigen::Affine3d& wT1, unsigned int object_id)
// {
//     boost::mutex::scoped_lock lock(collision_world_mutex_);
//     return fclInterface.addCollisionObject(s1, wT1, object_id);
// }

// bool ConstrainedManipulability::addCollisionObject(const shape_msgs::Mesh& s1,
//                                                     const Eigen::Affine3d& wT1, unsigned int object_id)
// {
//     boost::mutex::scoped_lock lock(collision_world_mutex_);
//     return fclInterface.addCollisionObject(s1, wT1, object_id);
// }

// bool ConstrainedManipulability::addCollisionObject(const octomap_msgs::Octomap& map,
//                                                     const Eigen::Affine3d& wT1, unsigned int object_id)
// {
//     boost::mutex::scoped_lock lock(collision_world_mutex_);
//     return fclInterface.addCollisionObject(map, wT1, object_id);
// }

// bool ConstrainedManipulability::addCollisionObject(robot_collision_checking::FCLObjectSet objects)
// {
//     boost::mutex::scoped_lock lock(collision_world_mutex_);
//     return fclInterface.addCollisionObject(objects);
// }

// bool ConstrainedManipulability::removeCollisionObject(unsigned int object_id)
// {
//     boost::mutex::scoped_lock lock(collision_world_mutex_);
//     return fclInterface.removeCollisionObject(object_id);
// }

// bool ConstrainedManipulability::displayObjects()
// {
//     boost::mutex::scoped_lock lock(collision_world_mutex_);
//     return fclInterface.displayObjects(base_link_);
// }

// bool ConstrainedManipulability::displayCollisionModel(sensor_msgs::JointState const& joint_states)
// {

//     KDL::JntArray kdl_joint_positions(ndof_);
//     jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);

//     GeometryInformation geometry_information;
//     // Collision Link transforms
//     getCollisionModel(kdl_joint_positions, geometry_information);
//     for (int i = 0; i < geometry_information.geometry_transforms.size(); ++i)
//     {
//         visualization_msgs::Marker mk;
//         shapes::constructMarkerFromShape(geometry_information.shapes[i].get(), mk, false);
//         displayMarker(mk, geometry_information.geometry_transforms[i], base_link_, i, {0.1, 0.5, 0.2, 0.5});
//     }

//     return false;
// }

// bool ConstrainedManipulability::displayMarker(visualization_msgs::Marker mkr,
//                                                 const Eigen::Affine3d& T,
//                                                 std::string frame,
//                                                 unsigned int obj_id,
//                                                 const Eigen::Vector4d& color)
// {
//     mkr.action = visualization_msgs::Marker::ADD;
//     mkr.header.frame_id = frame;
//     mkr.ns = "Objects";
//     mkr.lifetime = ros::Duration(0.0);
//     mkr.id = obj_id;
//     mkr.color.r = color(0);
//     mkr.color.g = color(1);
//     mkr.color.b = color(2);
//     mkr.color.a = color(3);

//     Eigen::Quaterniond q(T.linear());
//     mkr.pose.position.x = T(0, 3);
//     mkr.pose.position.y = T(1, 3);
//     mkr.pose.position.z = T(2, 3);
//     mkr.pose.orientation.w = q.w();
//     mkr.pose.orientation.x = q.x();
//     mkr.pose.orientation.y = q.y();
//     mkr.pose.orientation.z = q.z();
//     mkr_pub_.publish(mkr);

//     return true;
// }

// void ConstrainedManipulability::getCartPos(const sensor_msgs::JointState& joint_states,
//                                             geometry_msgs::Pose& geo_pose)
// {

//     KDL::JntArray kdl_joint_positions(ndof_);
//     jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);

//     KDL::Frame cartpos;
//     kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos);
//     Eigen::Affine3d T;
//     tf2::transformKDLToEigen(cartpos, T);
//     geo_pose = tf2::toMsg(T);
// }

// bool ConstrainedManipulability::checkCollision(const sensor_msgs::JointState& joint_states)
// {
//     KDL::JntArray kdl_joint_positions(ndof_);
//     jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);

//     GeometryInformation geometry_information;
//     getCollisionModel(kdl_joint_positions, geometry_information);

//     std::vector<shapes::ShapeMsg> current_shapes;
//     std::vector<geometry_msgs::Pose> shapes_poses;

//     convertCollisionModel(geometry_information, current_shapes, shapes_poses);

//     boost::mutex::scoped_lock lock(collision_world_mutex_);

//     std::vector<geometry_msgs::PoseStamped> shapes_poses_stamped;
//     for (int i = 0; i < shapes_poses.size(); i++)
//     {
//         geometry_msgs::PoseStamped shape_stamped;
//         shape_stamped.header.stamp = joint_states.header.stamp;
//         shape_stamped.header.frame_id = base_link_;
//         shape_stamped.pose = shapes_poses[i];
//         shapes_poses_stamped.push_back(shape_stamped);
//     }

//     // If octomap is available, then filter robot from octomap and add to collision world
//     if (octo_filter_->addObjectToOctoFilter(current_shapes, shapes_poses_stamped))
//     {
//         // Get the octomap properties, including its pose in the robot base frame
//         octomap_msgs::Octomap octomap;
//         geometry_msgs::TransformStamped octomap_wrt_base;
//         octo_filter_->getOctomapProperties(base_link_, octomap, octomap_wrt_base);
//         Eigen::Affine3d octomap_pose_wrt_base;
//         tf2::transformMsgToEigen(octomap_wrt_base.transform, octomap_pose_wrt_base);

//         // Remove the old octomap from the world
//         fclInterface.removeCollisionObject(octomap_id_);
//         // Update with the new
//         fclInterface.addCollisionObject(octomap, octomap_pose_wrt_base, octomap_id_);
//     }

//     for (int i = 0; i < geometry_information.geometry_transforms.size(); i++)
//     {
//         if (current_shapes[i].which() == 0)
//         {
//             if (fclInterface.checkCollisionObjectWorld(boost::get<shape_msgs::SolidPrimitive>(current_shapes[i]),
//                                                         geometry_information.geometry_transforms[i]))
//             {
//                 return true;
//             }
//         }
//         else if (current_shapes[i].which() == 1)
//         {
//             if (fclInterface.checkCollisionObjectWorld(boost::get<shape_msgs::Mesh>(current_shapes[i]),
//                                                         geometry_information.geometry_transforms[i]))
//             {
//                 return true;
//             }
//         }
//         else
//         {
//             ROS_ERROR("Collision Geometry not support");
//         }
//     }

//     return false;
// }

void ConstrainedManipulability::getJacobian(const sensor_msgs::msg::JointState& joint_states, Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    Eigen::Affine3d base_T_ee;
    jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, base_T_ee, Jac);
}

void ConstrainedManipulability::getTransform(const sensor_msgs::msg::JointState& joint_states, Eigen::Affine3d& T) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> Jac;
    jointStatetoKDLJointArray(chain_, joint_states, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, T, Jac);
}

/// Private member methods (excluding ROS callbacks)

std::unique_ptr<shapes::Shape> ConstrainedManipulability::constructShape(const urdf::Geometry* geom) const
{
    assert(geom != nullptr);

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
            RCLCPP_WARN(this->get_logger(), "Empty mesh filename");
        }
        break;
    }
    default:
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown geometry type: %d", (int)geom->type);
        break;
    }
    }
    return (result);
}

void ConstrainedManipulability::getKDLKinematicInformation(const KDL::JntArray& kdl_joint_positions, Eigen::Affine3d& T,
                                                           Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac, int segment) const
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
    tf2::transformKDLToEigen(cartpos, T);

    Jac = base_J_link_origin.data;
}

void ConstrainedManipulability::getCollisionModel(const KDL::JntArray& kdl_joint_positions, GeometryInformation& geometry_information) const
{
    geometry_information.clear();
    Eigen::Affine3d link_origin_T_collision_origin, base_T_link_origin, base_T_collision_origin;

    // Calculates the segment's collision geomtery
    // The transform to the origin of the collision geometry
    // The Jacobian matrix at the origin of the collision geometry
    int num_segments = chain_.getNrOfSegments();
    for (int i = 0; i < num_segments; ++i)
    {
        KDL::Segment seg = chain_.getSegment(i); // Get current segment

        // Get Collision Geometry
        std::unique_ptr<shapes::Shape> shape = constructShape(model_->links_.at(seg.getName())->collision->geometry.get());

        // Get Collision Origin
        Eigen::Vector3d origin_Trans_collision(model_->links_.at(seg.getName())->collision->origin.position.x,
                                                model_->links_.at(seg.getName())->collision->origin.position.y,
                                                model_->links_.at(seg.getName())->collision->origin.position.z);
        Eigen::Quaterniond origin_Quat_collision(
            model_->links_.at(seg.getName())->collision->origin.rotation.w,
            model_->links_.at(seg.getName())->collision->origin.rotation.x,
            model_->links_.at(seg.getName())->collision->origin.rotation.y,
            model_->links_.at(seg.getName())->collision->origin.rotation.z);

        link_origin_T_collision_origin.translation() = origin_Trans_collision;
        link_origin_T_collision_origin.linear() = origin_Quat_collision.toRotationMatrix();

        // Finds cartesian pose w.r.t to base frame
        Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_collision_origin, base_J_link_origin;
        getKDLKinematicInformation(kdl_joint_positions, base_T_link_origin, base_J_link_origin, i + 1);
        base_T_collision_origin = base_T_link_origin * link_origin_T_collision_origin;
        Eigen::Vector3d base_L_link_collision = (base_T_link_origin.linear() * link_origin_T_collision_origin.translation());
        // Screw transform to collision origin
        screwTransform(base_J_link_origin, base_L_link_collision, base_J_collision_origin);

        // Push back solutions
        geometry_information.shapes.push_back(std::move(shape));
        geometry_information.geometry_transforms.push_back(base_T_collision_origin);
        geometry_information.geometry_jacobians.push_back(base_J_collision_origin);
    }
}

void ConstrainedManipulability::convertCollisionModel(const GeometryInformation& geometry_information,
                                                      std::vector<shapes::ShapeMsg>& current_shapes,
                                                      std::vector<geometry_msgs::msg::Pose>& shapes_poses) const
{
    int num_transforms = geometry_information.geometry_transforms.size();

    current_shapes.resize(num_transforms);
    shapes_poses.resize(num_transforms);

    for (int i = 0; i < num_transforms; i++)
    {
        shapes::ShapeMsg current_shape;
        shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shapes[i]);
        
        shapes_poses[i] = tf2::toMsg(geometry_information.geometry_transforms[i]);
    }
}
} // namespace constrained_manipulability