#include <cassert>
#include <chrono>

#include <rclcpp/logging.hpp>

#include <geometric_shapes/shape_to_marker.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "constrained_manipulability_interfaces/msg/matrix.hpp"

#include "constrained_manipulability/constrained_manipulability.hpp"

namespace constrained_manipulability
{
ConstrainedManipulability::ConstrainedManipulability(const rclcpp::NodeOptions& options) : Node("constrained_manipulability", options)
{
    // Populate private properties
    base_link_ = this->declare_parameter<std::string>("root", "base_link");
    tip_ = this->declare_parameter<std::string>("tip", "ee_link");

    distance_threshold_ = this->declare_parameter<double>("distance_threshold", 0.3);
    dangerfield_ = this->declare_parameter<double>("dangerfield", 10.0);

    double lin_limit = this->declare_parameter<double>("linearization_limit", 0.1);

    publish_mp_ = this->declare_parameter<bool>("publish_mp", true);
    publish_cmp_ = this->declare_parameter<bool>("publish_cmp", true);
    publish_vp_ = this->declare_parameter<bool>("publish_vp", true);
    publish_cvp_ = this->declare_parameter<bool>("publish_cvp", true);

    show_mp_ = this->declare_parameter<bool>("show_mp", true);
    show_cmp_ = this->declare_parameter<bool>("show_cmp", true);
    show_vp_ = this->declare_parameter<bool>("show_vp", false);
    show_cvp_ = this->declare_parameter<bool>("show_cvp", false);

    filter_robot_ = this->declare_parameter<bool>("filter_robot", false);

    // TF properties
    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    // Can set robot_description name from parameters
    std::string robot_description_name = "robot_description";
    this->get_parameter_or("robot_description_name", robot_description_name, robot_description_name);

    // Create collision world
    collision_world_ = std::make_shared<robot_collision_checking::FCLInterfaceCollisionWorld>(base_link_);

    // Create parameter client to grab the robot_description from another node (robot_state_publisher)
    auto params_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
    while (!params_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_FATAL(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Robot state description service not available, waiting...");
    }

    if (!params_client->has_parameter(robot_description_name))
    {
        RCLCPP_FATAL(this->get_logger(), "Parameter %s not found in node robot_state_publisher", robot_description_name.c_str());
        rclcpp::shutdown();
    }

    // Grab URDF robot description
    auto parameters = params_client->get_parameters({ robot_description_name });
    std::string robot_desc_string = parameters[0].value_to_string();

    // Initialize URDF model
    model_ = std::make_unique<urdf::Model>();

    // Verify that URDF string is in correct format
    if (!model_->initString(robot_desc_string))
    {
        RCLCPP_FATAL(this->get_logger(),"URDF string is not a valid robot model.");
        rclcpp::shutdown();
    }

    // Robot kinematics model creation as KDL tree using URDF model
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(*model_, tree))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
    }

    tree.getChain(base_link_, tip_, chain_);
    ndof_ = chain_.getNrOfJoints();
    int nseg = chain_.getNrOfSegments();

    RCLCPP_INFO(this->get_logger(), "Loading tree from parameter %s with kinematic chain from %s to %s", robot_description_name.c_str(), base_link_.c_str(), tip_.c_str());
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

    // Set active joint states as a parameter
    this->declare_parameter("active_dof", std::vector<std::string>{});
    this->set_parameter(rclcpp::Parameter("active_dof", joint_names));

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

    // Instantiate ROS services and subscribers/publishers
    mesh_coll_server_ = this->create_service<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh>(
        "add_remove_collision_mesh",  std::bind(&ConstrainedManipulability::addRemoveMeshCallback, this, std::placeholders::_1, std::placeholders::_2));
    solid_coll_server_ = this->create_service<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid>(
        "add_remove_collision_solid",  std::bind(&ConstrainedManipulability::addRemoveSolidCallback, this, std::placeholders::_1, std::placeholders::_2));
    jacobian_server_ = this->create_service<constrained_manipulability_interfaces::srv::GetJacobianMatrix>(
        "get_jacobian_matrix",  std::bind(&ConstrainedManipulability::getJacobianCallback, this, std::placeholders::_1, std::placeholders::_2));
    polytopes_server_ = this->create_service<constrained_manipulability_interfaces::srv::GetPolytopes>(
        "get_polytopes",  std::bind(&ConstrainedManipulability::getPolytopesCallback, this, std::placeholders::_1, std::placeholders::_2));
    sliced_polytope_server_ = this->create_service<constrained_manipulability_interfaces::srv::GetSlicedPolytope>(
        "get_sliced_polytope",  std::bind(&ConstrainedManipulability::getSlicedPolytopeCallback, this, std::placeholders::_1, std::placeholders::_2));

    rclcpp::SubscriptionOptions joint_sub_options;
    joint_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions octo_sub_options;
    octo_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions lin_lim_sub_options;
    lin_lim_sub_options.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(1), 
        std::bind(&ConstrainedManipulability::jointStateCallback, this, std::placeholders::_1),
        joint_sub_options);
    octomap_filter_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_filtered", rclcpp::QoS(1), 
        std::bind(&ConstrainedManipulability::octomapCallback, this, std::placeholders::_1),
        octo_sub_options);
    lin_limit_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/lin_limit", rclcpp::QoS(1), 
        std::bind(&ConstrainedManipulability::linLimitCallback, this, std::placeholders::_1),
        lin_lim_sub_options);

    coll_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&ConstrainedManipulability::checkCollisionCallback, this));
    polytope_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250),
            std::bind(&ConstrainedManipulability::polytopePubCallback, this));

    mkr_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 1);
    obj_dist_pub_ = this->create_publisher<constrained_manipulability_interfaces::msg::ObjectDistances>("constrained_manipulability/obj_distances", 1);
    poly_pub_ = this->create_publisher<constrained_manipulability_interfaces::msg::Polytope>("constrained_manipulability/polytope", 1);
    filt_mesh_pub_ = this->create_publisher<octomap_filter_interfaces::msg::FilterMesh>("filter_mesh", 1);
    filt_prim_pub_ = this->create_publisher<octomap_filter_interfaces::msg::FilterPrimitive>("filter_primitive", 1);

    RCLCPP_INFO(this->get_logger(), "Initialized constrained_manipulability");
}

/// ROS interface methods

void ConstrainedManipulability::addRemoveMeshCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh::Request> req,
                                                      std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh::Response> res)
{
    res->result = false;

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    if (req->remove)
    {
        removeCollisionObject(req->object_id);
    }
    else
    {
        Eigen::Affine3d mesh_T;
        tf2::fromMsg(req->pose, mesh_T);

        robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
            req->mesh, robot_collision_checking::MESH, mesh_T);
        addCollisionObject(obj, req->object_id);
    }

    res->result = true;
}

void ConstrainedManipulability::addRemoveSolidCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid::Request> req,
                                                       std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid::Response> res)
{
    res->result = false;

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    if (req->remove)
    {
        removeCollisionObject(req->object_id);
    }
    else
    {
        Eigen::Affine3d solid_T;
        tf2::fromMsg(req->pose, solid_T);

        robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(req->solid, solid_T);
        addCollisionObject(obj, req->object_id);
    }
}

void ConstrainedManipulability::getJacobianCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetJacobianMatrix::Request> req,
                                                    std::shared_ptr<constrained_manipulability_interfaces::srv::GetJacobianMatrix::Response> res)
{
    Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
    getJacobian(req->joint_state, base_J_ee);
    res->jacobian = eigenToMatrix(base_J_ee);
}

void ConstrainedManipulability::getPolytopesCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetPolytopes::Request> req,
                                                     std::shared_ptr<constrained_manipulability_interfaces::srv::GetPolytopes::Response> res)
{
    int num_states = req->joint_states.size();
    res->polytopes.resize(num_states);
    for (int i = 0; i < num_states; ++i)
    {
        Eigen::MatrixXd AHrep;
        Eigen::VectorXd bHrep;
        Eigen::Vector3d offset_position;
        // Safe to assign, no pointer members
        Polytope poly;
        if (req->polytopes_type == req->ALLOWABLE_MOTION_POLYTOPE)
        {
            poly = getAllowableMotionPolytope(req->joint_states[i], req->show_polytopes, AHrep, bHrep, offset_position);
        }
        else if (req->polytopes_type == req->CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE)
        {
            poly = getConstrainedAllowableMotionPolytope(req->joint_states[i], req->show_polytopes, AHrep, bHrep, offset_position);
        }
        else if (req->polytopes_type == req->VELOCITY_POLYTOPE)
        {
            poly = getVelocityPolytope(req->joint_states[i], req->show_polytopes, AHrep, bHrep, offset_position);
        }
        else if (req->polytopes_type == req->CONSTRAINED_VELOCITY_POLYTOPE)
        {
            poly = getConstrainedVelocityPolytope(req->joint_states[i], req->show_polytopes, AHrep, bHrep, offset_position);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown Polytope type");
            poly = Polytope();
        }

        constrained_manipulability_interfaces::msg::Polytope poly_msg;
        poly_msg.stamp = req->joint_states[i].header.stamp;
        poly_msg.name = poly.getName();
        poly_msg.volume = poly.getVolume();
        if (poly.isValid())
        {
            poly_msg.mesh = poly.getMesh();
            poly_msg.hyperplanes = eigenToMatrix(AHrep);
            poly_msg.shifted_distance =  eigenToVector(bHrep);
        }

        res->polytopes[i] = poly_msg;
    }
}

void ConstrainedManipulability::getSlicedPolytopeCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetSlicedPolytope::Request> req,
                                                          std::shared_ptr<constrained_manipulability_interfaces::srv::GetSlicedPolytope::Response> res)
{
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bHrep;
    Eigen::Vector3d offset_position;

    // Safe to assign, no pointer members
    Polytope poly;
    if (req->polytope_type == req->ALLOWABLE_MOTION_POLYTOPE)
    {
        poly = getAllowableMotionPolytope(req->joint_state, false, AHrep, bHrep, offset_position);
    }
    else if (req->polytope_type == req->CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE)
    {
        poly = getConstrainedAllowableMotionPolytope(req->joint_state, false, AHrep, bHrep, offset_position);
    }
    else if (req->polytope_type == req->VELOCITY_POLYTOPE)
    {
        poly = getVelocityPolytope(req->joint_state, false, AHrep, bHrep, offset_position);
    }
    else if (req->polytope_type == req->CONSTRAINED_VELOCITY_POLYTOPE)
    {
        poly = getConstrainedVelocityPolytope(req->joint_state, false, AHrep, bHrep, offset_position);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unknown Polytope type");
        poly = Polytope();
    }

    Polytope sliced_poly;
    if (poly.isValid())
    {
        std::vector<double> color_pts;
        std::vector<double> color_line;
        if (req->slicing_plane == req->YZ_PLANE)
        {
            sliced_poly = poly.slice("yz_slice", SLICING_PLANE::YZ_PLANE, req->plane_width);
            color_pts = {1.0, 0.0, 0.0, 1.0};
            color_line = {1.0, 0.0, 0.0, 0.4};
        }
        else if (req->slicing_plane == req->XZ_PLANE)
        {
            sliced_poly = poly.slice("xz_slice", SLICING_PLANE::XZ_PLANE, req->plane_width);
            color_pts = {1.0, 1.0, 0.0, 1.0};
            color_line = {1.0, 1.0, 0.0, 0.4};
        }
        else
        {
            sliced_poly = poly.slice("xy_slice", SLICING_PLANE::XY_PLANE, req->plane_width);
            color_pts = {0.0, 1.0, 0.0, 1.0};
            color_line = {0.0, 1.0, 0.0, 0.4};
        }

        if (sliced_poly.isValid())
        {
            plotPolytope(sliced_poly.getName(), sliced_poly.getPoints(), color_pts, color_line);
        }
    }

    constrained_manipulability_interfaces::msg::Polytope poly_msg;
    poly_msg.stamp = req->joint_state.header.stamp;
    poly_msg.name = sliced_poly.getName();
    poly_msg.volume = sliced_poly.getVolume();
    if (sliced_poly.isValid())
    {
        poly_msg.mesh = sliced_poly.getMesh();
        poly_msg.hyperplanes = eigenToMatrix(AHrep);
        poly_msg.shifted_distance =  eigenToVector(bHrep);
    }

    res->polytope = poly_msg;
}

void ConstrainedManipulability::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_state_mutex_.lock();
    joint_state_ = *msg;
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state_, kdl_joint_positions);
    joint_state_mutex_.unlock();

    GeometryInformation geometry_information;
    getCollisionModel(kdl_joint_positions, geometry_information);

    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    if (filter_robot_)
    {
        int num_shapes = current_shapes.size();
        for (int i = 0; i < num_shapes; i++)
        {
            geometry_msgs::msg::PoseStamped shape_stamped;
            shape_stamped.header.stamp = msg->header.stamp;
            shape_stamped.header.frame_id = base_link_;
            shape_stamped.pose = shapes_poses[i];
            // Filter robot from octomap
            if (current_shapes[i].which() == 0)
            {
                octomap_filter_interfaces::msg::FilterPrimitive filter_primitive;
                filter_primitive.primitive = boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]);
                filter_primitive.pose = shape_stamped;
                filt_prim_pub_->publish(filter_primitive);
            }
            else if (current_shapes[i].which() == 1)
            {
                octomap_filter_interfaces::msg::FilterMesh filter_mesh;
                filter_mesh.mesh = boost::get<shape_msgs::msg::Mesh>(current_shapes[i]);
                filter_mesh.pose = shape_stamped;
                filt_mesh_pub_->publish(filter_mesh);
            }
        }
    }

    displayCollisionModel(geometry_information, {0.1, 0.5, 0.2, 0.5});
 }

void ConstrainedManipulability::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    // Get octomap pose w.r.t. the robot base frame
    geometry_msgs::msg::TransformStamped octomap_wrt_base;
    try
    {
        octomap_wrt_base = buffer_->lookupTransform(
            base_link_,
            msg->header.frame_id,
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s",
            base_link_.c_str(), msg->header.frame_id.c_str(), ex.what());                
    }

    Eigen::Affine3d octomap_pose_wrt_base = tf2::transformToEigen(octomap_wrt_base.transform);

    boost::mutex::scoped_lock lock(collision_world_mutex_);
    // Remove the old octomap from the world
    collision_world_->removeCollisionObject(OCTOMAP_ID);
    // Update with the new octomap
    robot_collision_checking::FCLObjectPtr octo_obj = std::make_shared<robot_collision_checking::FCLObject>(
        *msg, robot_collision_checking::OCTOMAP, octomap_pose_wrt_base);
    // Add the filtered octomap to the collision world
    collision_world_->addCollisionObject(octo_obj, OCTOMAP_ID);
}

void ConstrainedManipulability::linLimitCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    setLinearizationLimit(msg->data);    
}

void ConstrainedManipulability::checkCollisionCallback()
{
    joint_state_mutex_.lock();
    sensor_msgs::msg::JointState curr_joint_state = joint_state_;
    joint_state_mutex_.unlock();
    
    checkCollision(curr_joint_state);
}

void ConstrainedManipulability::polytopePubCallback()
{
    joint_state_mutex_.lock();
    sensor_msgs::msg::JointState curr_joint_state = joint_state_;
    joint_state_mutex_.unlock();

    if (publish_mp_)
    {
        Polytope allowable_poly = getAllowableMotionPolytope(curr_joint_state, show_mp_);
    }

    if (publish_cmp_)
    {
        Polytope constrained_motion_poly = getConstrainedAllowableMotionPolytope(curr_joint_state, show_cmp_);
    }

    if (publish_vp_)
    {
        Polytope velocity_poly = getVelocityPolytope(curr_joint_state, show_vp_);
    }

    if (publish_cvp_)
    {
        Polytope constrained_velocity_poly = getConstrainedVelocityPolytope(curr_joint_state, show_cvp_);
    }
}

/// Private member methods (excluding ROS callbacks)

bool ConstrainedManipulability::addCollisionObject(const robot_collision_checking::FCLObjectPtr& obj, int object_id)
{
    return collision_world_->addCollisionObject(obj, object_id);
}

bool ConstrainedManipulability::removeCollisionObject(int object_id)
{
    return collision_world_->removeCollisionObject(object_id);
}

bool ConstrainedManipulability::getPolytopeHyperPlanes(const KDL::JntArray& kdl_joint_positions,
                                                       const GeometryInformation& geometry_information,
                                                       const builtin_interfaces::msg::Time& joint_state_stamp,
                                                       Eigen::MatrixXd& AHrep,
                                                       Eigen::VectorXd& bHrep,
                                                       bool velocity_polytope)
{
    // Closest points
    std::vector<Eigen::Vector3d> p1w, p2w;
    // Min distance to collision object
    std::vector<double> obj_distances;
    // Vector towards the collision object
    Eigen::Vector3d nt;
    // Vector of distance
    std::vector<double> distances;
    std::vector<Eigen::Matrix<double, 1, Eigen::Dynamic>> J_constraints;

    // Lock the collision world
    boost::mutex::scoped_lock lock(collision_world_mutex_);
    // For all robot links
    int num_shapes = geometry_information.shapes.size();
    for (int i = 0; i < num_shapes; i++)
    {
        shapes::ShapeMsg current_shape;
        shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shape);

        if (current_shape.which() == 0)
        {
            shape_msgs::msg::SolidPrimitive solid = boost::get<shape_msgs::msg::SolidPrimitive>(current_shape);
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(solid, geometry_information.geometry_transforms[i]);

            fcl::Transform3d world_to_fcl;
            robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
            robot_collision_checking::FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);
            
            collision_world_->getObjectDistances(co, obj_distances, p1w, p2w);
        }
        else if (current_shape.which() == 1)
        {
            shape_msgs::msg::Mesh mesh = boost::get<shape_msgs::msg::Mesh>(current_shape);
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                mesh, robot_collision_checking::MESH, geometry_information.geometry_transforms[i]);

            fcl::Transform3d world_to_fcl;
            robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
            robot_collision_checking::FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);

            collision_world_->getObjectDistances(co, obj_distances, p1w, p2w);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Collision geometry not supported");
        }

        for (unsigned int j = 0; j < obj_distances.size(); j++)
        {
            Eigen::Matrix<double, 6, Eigen::Dynamic> w_J_out_p1;
            Eigen::Matrix<double, 1, Eigen::Dynamic> J_proj;
            w_J_out_p1.setZero();
            J_proj.setZero();

            if (obj_distances[j] < 0.0)
            {
                // RCLCPP_WARN(this->get_logger(), "Robot in collision");
                return false;
            }
            else if (obj_distances[j] < distance_threshold_)
            {
                Eigen::Vector3d rdiff;
                if (collision_world_->getCollisionObjects()[j]->collision_id == OCTOMAP_ID)
                {
                    rdiff = p1w[j] - p2w[j]; // Backwards octomap vector
                }
                else
                {
                    rdiff = p2w[j] - p1w[j];
                }
                nt = rdiff; // Direction of obstacle
                nt.normalize();

                Eigen::Vector3d w_delta_p1_collision_origin = p1w[j] - geometry_information.geometry_transforms[i].translation();

                screwTransform(geometry_information.geometry_jacobians[i], w_delta_p1_collision_origin,  w_J_out_p1);
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

    // For velocity polytopes there are ndof*2 less hyperplanes
    int offset(4);
    if (velocity_polytope)
    {
        offset = 2;
    }
    // Define Hyperplanes
    AHrep.resize(offset * ndof_ + J_constraints.size(),
                    ndof_);
    bHrep.resize(offset * ndof_ + distances.size(), 1);
    AHrep.setZero();
    bHrep.setZero();

    if (velocity_polytope)
    {                                                                 // If velocity, then the joint position constraints simply scale max velocity
        AHrep.topRows(ndof_) = ndof_identity_matrix_;                 // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

        for (unsigned int i = 0; i < ndof_; ++i)
        {

            double qmean = (qmax_[i] + qmin_[i]) / 2;
            double val_max = fmax(qmean, kdl_joint_positions(i)) - qmean;
            double val_min = fmin(qmean, kdl_joint_positions(i)) - qmean;
            double dmax = pow((((val_max) / ((qmax_[i] - qmean)))), 2);
            double dmin = pow((((val_min) / ((qmin_[i] - qmean)))), 2);

            // Make sure the value is witin joint limits and these limits are correctly defined
            assert(!std::isnan(dmax) && !std::isnan(dmin) && !std::isinf(dmax) && !std::isinf(dmin));
            // Scale the maximum joint velocity based on joint position limits
            bHrep(i) = (1 - dmax) * qdotmax_[i];
            bHrep(i + ndof_) = (1 - dmin) * -qdotmin_[i];
        }
    }
    else
    {
        AHrep.topRows(ndof_) = ndof_identity_matrix_;                     // ndof_*ndof block at row 0 colum 0;ndof_
        AHrep.block(ndof_, 0, ndof_, ndof_) = ndof_identity_matrix_;      // ndof_*ndof block at row ndof_ colum 0;ndof_
        AHrep.block(ndof_ * 2, 0, ndof_, ndof_) = ndof_identity_matrix_;  // ndof_*ndof block at row ndof_*2 colum 0;ndof_
        AHrep.block(ndof_ * 3, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row ndof_*3 colum 0;ndof_ -ndof_identity_matrix_ for negative joint limits
        for (unsigned int i = 0; i < ndof_; ++i)
        {
            bHrep(i) = qmax_[i] - kdl_joint_positions(i);
            bHrep(i + ndof_) = kdl_joint_positions(i) - qmin_[i];
            bHrep(i + 2 * ndof_) = max_lin_limit_[i];
            bHrep(i + 3 * ndof_) = -min_lin_limit_[i];
        }
    }

    constrained_manipulability_interfaces::msg::ObjectDistances dist_arr;
    for (unsigned int i = 0; i < J_constraints.size(); ++i)
    {
        AHrep.row(offset * ndof_ + i) = J_constraints[i];
        bHrep(offset * ndof_ + i) = distances[i]; // Small tolerance to stop passing through
        dist_arr.distances.push_back(distances[i]);
    }

    dist_arr.stamp = joint_state_stamp;
    obj_dist_pub_->publish(dist_arr);

    return true;
}

Polytope ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                               bool show_polytope,
                                                               Eigen::MatrixXd& AHrep,
                                                               Eigen::VectorXd& bHrep,
                                                               Eigen::Vector3d& offset_position,
                                                               const std::vector<double>& color_pts,
                                                               const std::vector<double>& color_line) const
{
    Eigen::Affine3d base_T_ee;
    Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
    KDL::JntArray kdl_joint_positions(ndof_);

    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, base_T_ee, base_J_ee);

    // Define Hyperplanes
    AHrep.resize(2 * ndof_, ndof_);
    AHrep.topRows(ndof_) = ndof_identity_matrix_;                    // ndof_*ndof block at row 0 colum 0;ndof_
    AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_;    // ndof_*ndof block at row ndof_ colum 0;ndof_

    // Define shifted distance from origin
    bHrep.resize(2 * ndof_, 1);
    for (unsigned int i = 0; i < ndof_; ++i)
    {
        bHrep(i) = max_lin_limit_[i];
        bHrep(i + ndof_) = -min_lin_limit_[i];
    }

    // Define offset position to end effector
    offset_position = base_T_ee.translation();

    // Convert to V-representation polytope
    Polytope vrep_polytope("allowable_motion_polytope", AHrep, bHrep, offset_position);

    // Transform to Cartesian Space
    vrep_polytope.transformCartesian(base_J_ee.topRows(3));

    // Publish polytope
    constrained_manipulability_interfaces::msg::Polytope poly_msg;
    poly_msg.stamp = joint_state.header.stamp;
    poly_msg.name = vrep_polytope.getName();
    poly_msg.mesh = vrep_polytope.getMesh();
    poly_msg.volume = vrep_polytope.getVolume();
    poly_msg.hyperplanes = eigenToMatrix(AHrep);
    poly_msg.shifted_distance =  eigenToVector(bHrep);
    poly_pub_->publish(poly_msg);

    if (show_polytope && vrep_polytope.isValid())
    {
        plotPolytope(poly_msg.name, vrep_polytope.getPoints(), color_pts, color_line);
    }

    // Return the calculated polytope
    return vrep_polytope;
}

Polytope ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                               bool show_polytope,
                                                               const std::vector<double>& color_pts,
                                                               const std::vector<double>& color_line) const
{
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
    Eigen::Vector3d offset_position;

    Polytope poly = getAllowableMotionPolytope(joint_state,
                                               show_polytope,
                                               AHrep,
                                               bhrep,
                                               offset_position,
                                               color_pts,
                                               color_line);
    return poly;
}

Polytope ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                                          bool show_polytope,
                                                                          Eigen::MatrixXd& AHrep,
                                                                          Eigen::VectorXd& bHrep,
                                                                          Eigen::Vector3d& offset_position,
                                                                          const std::vector<double>& color_pts,
                                                                          const std::vector<double>& color_line)
{
    GeometryInformation geometry_information;
    KDL::JntArray kdl_joint_positions(ndof_);

    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getCollisionModel(kdl_joint_positions, geometry_information);

    // Build collision geometry for robot if not yet created
    // Assume collision geometry for robot's meshes do not change
    if (robot_collision_geometry_.size() == 0)
    {
        createRobotCollisionModel(geometry_information);
    }
    
    bool collision_free = getPolytopeHyperPlanes(kdl_joint_positions, geometry_information, joint_state.header.stamp, AHrep, bHrep);
    if (!collision_free)
    {
        return Polytope();
    }

    // Define offset position based on geometrical collision world for robot body
    offset_position = geometry_information.geometry_transforms.back().translation();

    // Convert to V-representation polytope
    Polytope vrep_polytope("constrained_allowable_motion_polytope", AHrep, bHrep, offset_position);
    // Transform to Cartesian Space
    vrep_polytope.transformCartesian(geometry_information.geometry_jacobians.back().topRows(3));

    // Publish polytope
    constrained_manipulability_interfaces::msg::Polytope poly_msg;
    poly_msg.stamp = joint_state.header.stamp;
    poly_msg.name = vrep_polytope.getName();
    poly_msg.mesh = vrep_polytope.getMesh();
    poly_msg.volume = vrep_polytope.getVolume();
    poly_msg.hyperplanes = eigenToMatrix(AHrep);
    poly_msg.shifted_distance =  eigenToVector(bHrep);
    poly_pub_->publish(poly_msg);

    if (show_polytope && vrep_polytope.isValid())
    {
        plotPolytope(poly_msg.name, vrep_polytope.getPoints(), color_pts, color_line);
    }

    // Return the calculated polytope
    return vrep_polytope;
}

Polytope ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                                          bool show_polytope,
                                                                          const std::vector<double>& color_pts,
                                                                          const std::vector<double>& color_line)
{
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
    Eigen::Vector3d offset_position;

    Polytope poly = getConstrainedAllowableMotionPolytope(joint_state,
                                                          show_polytope,
                                                          AHrep,
                                                          bhrep,
                                                          offset_position,
                                                          color_pts,
                                                          color_line);
    return poly;
}

Polytope ConstrainedManipulability::getVelocityPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                        bool show_polytope,
                                                        Eigen::MatrixXd& AHrep,
                                                        Eigen::VectorXd& bHrep,
                                                        Eigen::Vector3d& offset_position,
                                                        const std::vector<double>& color_pts,
                                                        const std::vector<double>& color_line) const
{
    Eigen::Affine3d base_T_ee;
    Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
    KDL::JntArray kdl_joint_positions(ndof_);

    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, base_T_ee, base_J_ee);

    // Define Hyperplanes
    AHrep.resize(2 * ndof_, ndof_);
    AHrep.topRows(ndof_) = ndof_identity_matrix_;                 // ndof_*ndof block at row  0 colum 0;ndof_
    AHrep.block(ndof_, 0, ndof_, ndof_) = -ndof_identity_matrix_; // ndof_*ndof block at row  ndof_ colum 0;ndof_

    // Define shifted distance from origin
    bHrep.resize(2 * ndof_, 1);
    for (unsigned int i = 0; i < ndof_; ++i)
    {
        bHrep(i) = qdotmax_[i];
        bHrep(i + ndof_) = -qdotmin_[i];
    }

    // Define offset position to end effector
    offset_position = base_T_ee.translation();

    // Convert to V-representation polytope
    Polytope vrep_polytope("velocity_polytope", AHrep, bHrep, offset_position);
    // Transform to Cartesian Space
    vrep_polytope.transformCartesian(base_J_ee.topRows(3));

    // Publish polytope
    constrained_manipulability_interfaces::msg::Polytope poly_msg;
    poly_msg.stamp = joint_state.header.stamp;
    poly_msg.name = vrep_polytope.getName();
    poly_msg.mesh = vrep_polytope.getMesh();
    poly_msg.volume = vrep_polytope.getVolume();
    poly_msg.hyperplanes = eigenToMatrix(AHrep);
    poly_msg.shifted_distance =  eigenToVector(bHrep);
    poly_pub_->publish(poly_msg);

    if (show_polytope && vrep_polytope.isValid())
    {
        plotPolytope(poly_msg.name, vrep_polytope.getPoints(), color_pts, color_line);
    }

    // Return the calculated polytope
    return vrep_polytope;
}

Polytope ConstrainedManipulability::getVelocityPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                        bool show_polytope,
                                                        const std::vector<double>& color_pts,
                                                        const std::vector<double>& color_line) const
{
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
    Eigen::Vector3d offset_position;

    Polytope poly = getVelocityPolytope(joint_state,
                                        show_polytope,
                                        AHrep,
                                        bhrep,
                                        offset_position,
                                        color_pts,
                                        color_line);
    return poly;
}

Polytope ConstrainedManipulability::getConstrainedVelocityPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                                   bool show_polytope,
                                                                   Eigen::MatrixXd& AHrep,
                                                                   Eigen::VectorXd& bHrep,
                                                                   Eigen::Vector3d& offset_position,
                                                                   const std::vector<double>& color_pts,
                                                                   const std::vector<double>& color_line)
{
    GeometryInformation geometry_information;
    KDL::JntArray kdl_joint_positions(ndof_);

    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getCollisionModel(kdl_joint_positions, geometry_information);

    // Build collision geometry for robot if not yet created
    // Assume collision geometry for robot's meshes do not change
    if (robot_collision_geometry_.size() == 0)
    {
        createRobotCollisionModel(geometry_information);
    }
    
    bool collision_free = getPolytopeHyperPlanes(kdl_joint_positions, geometry_information, joint_state.header.stamp, AHrep, bHrep, true);
    if (!collision_free)
    {
        return Polytope();
    }

    // Define offset position based on geometrical collision world for robot body
    offset_position = geometry_information.geometry_transforms.back().translation();

    // Convert to V-representation polytope
    Polytope vrep_polytope("constrained_velocity_polytope", AHrep, bHrep, offset_position);
    // Transform to Cartesian Space
    vrep_polytope.transformCartesian(geometry_information.geometry_jacobians.back().topRows(3));

    // Publish polytope
    constrained_manipulability_interfaces::msg::Polytope poly_msg;
    poly_msg.stamp = joint_state.header.stamp;
    poly_msg.name = vrep_polytope.getName();
    poly_msg.mesh = vrep_polytope.getMesh();
    poly_msg.volume = vrep_polytope.getVolume();
    poly_msg.hyperplanes = eigenToMatrix(AHrep);
    poly_msg.shifted_distance =  eigenToVector(bHrep);
    poly_pub_->publish(poly_msg);

    if (show_polytope && vrep_polytope.isValid())
    {
        plotPolytope(poly_msg.name, vrep_polytope.getPoints(), color_pts, color_line);
    }

    // Return the calculated polytope
    return vrep_polytope;
}

Polytope ConstrainedManipulability::getConstrainedVelocityPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                                   bool show_polytope,
                                                                   const std::vector<double>& color_pts,
                                                                   const std::vector<double>& color_line)
{
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
    Eigen::Vector3d offset_position;

    Polytope poly = getConstrainedVelocityPolytope(joint_state,
                                                   show_polytope,
                                                   AHrep,
                                                   bhrep,
                                                   offset_position,
                                                   color_pts,
                                                   color_line);
    return poly;
}


void ConstrainedManipulability::plotPolytope(const std::string& poly_name, const std::vector<geometry_msgs::msg::Point>& points,
                                             const std::vector<double>& color_pts, const std::vector<double>& color_line) const
{
    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

    visualization_msgs::msg::Marker mkr_mesh;
    mkr_mesh.ns = poly_name;
    mkr_mesh.action = visualization_msgs::msg::Marker::ADD;
    mkr_mesh.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    mkr_mesh.header.frame_id = base_link_;
    mkr_mesh.id = 2;
    mkr_mesh.lifetime = rclcpp::Duration(0, 0);
    mkr_mesh.color.r = color_line[0];
    mkr_mesh.color.g = color_line[1];
    mkr_mesh.color.b = color_line[2];
    mkr_mesh.color.a = color_line[3];
    mkr_mesh.scale.x = 1.0;
    mkr_mesh.scale.y = 1.0;
    mkr_mesh.scale.z = 1.0;
    mkr_mesh.points = points;
    marker_array_msg->markers.push_back(mkr_mesh);

    visualization_msgs::msg::Marker mkr_sphere;
    mkr_sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mkr_sphere.header.frame_id = base_link_;
    mkr_sphere.id = 1;
    mkr_sphere.lifetime = rclcpp::Duration(0, 0);
    mkr_sphere.color.r = color_pts[0];
    mkr_sphere.color.g = color_pts[1];
    mkr_sphere.color.b = color_pts[2];
    mkr_sphere.color.a = color_pts[3];
    mkr_sphere.scale.x = 0.005;
    mkr_sphere.scale.y = 0.005;
    mkr_sphere.scale.z = 0.005;
    mkr_sphere.points = points;
    marker_array_msg->markers.push_back(mkr_sphere);

    mkr_pub_->publish(*marker_array_msg);
}

bool ConstrainedManipulability::checkCollision(const sensor_msgs::msg::JointState& joint_state)
{
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);

    GeometryInformation geometry_information;
    getCollisionModel(kdl_joint_positions, geometry_information);

    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    // Build collision geometry for robot if not yet created
    // Assume collision geometry for robot's meshes do not change
    if (robot_collision_geometry_.size() == 0)
    {
        createRobotCollisionModel(geometry_information);
    }
    
    boost::mutex::scoped_lock lock(collision_world_mutex_);
    int num_shapes = geometry_information.shapes.size();
    for (int i = 0; i < num_shapes; i++)
    {
        Eigen::Affine3d obj_pose;
        tf2::fromMsg(shapes_poses[i], obj_pose);
        std::vector<int> collision_object_ids;
        if (current_shapes[i].which() == 0)
        {
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]), obj_pose);

            fcl::Transform3d world_to_fcl;
            robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
            robot_collision_checking::FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);
            
            if (collision_world_->checkCollisionObject(co, collision_object_ids))
            {
                return true;
            }
        }
        else if (current_shapes[i].which() == 1)
        {
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::Mesh>(current_shapes[i]), robot_collision_checking::MESH, obj_pose);

            fcl::Transform3d world_to_fcl;
            robot_collision_checking::fcl_interface::transform2fcl(obj->object_transform, world_to_fcl);
            robot_collision_checking::FCLCollisionObjectPtr co = std::make_shared<fcl::CollisionObjectd>(robot_collision_geometry_[i], world_to_fcl);

            if (collision_world_->checkCollisionObject(co, collision_object_ids))
            {
                return true;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Collision geometry not supported");
        }
    }
    
    return false;
}

void ConstrainedManipulability::displayCollisionModel(const GeometryInformation& geometry_information, const Eigen::Vector4d& color) const
{
    auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

    int num_shapes = geometry_information.shapes.size();
    for (int i = 0; i < num_shapes; ++i)
    {
        visualization_msgs::msg::Marker mkr;
        shapes::constructMarkerFromShape(geometry_information.shapes[i].get(), mkr);
        mkr.ns = "collision_body";
        mkr.header.frame_id = base_link_;
        mkr.action = visualization_msgs::msg::Marker::ADD;
        mkr.lifetime = rclcpp::Duration(0, 0);
        mkr.id = i;
        mkr.color.r = color(0);
        mkr.color.g = color(1);
        mkr.color.b = color(2);
        mkr.color.a = color(3);

        Eigen::Quaterniond q(geometry_information.geometry_transforms[i].linear());
        mkr.pose.position.x = geometry_information.geometry_transforms[i](0, 3);
        mkr.pose.position.y = geometry_information.geometry_transforms[i](1, 3);
        mkr.pose.position.z = geometry_information.geometry_transforms[i](2, 3);
        mkr.pose.orientation.w = q.w();
        mkr.pose.orientation.x = q.x();
        mkr.pose.orientation.y = q.y();
        mkr.pose.orientation.z = q.z();

        marker_array_msg->markers.push_back(mkr);
    }
    
    std::vector<robot_collision_checking::FCLInterfaceCollisionObjectPtr> world_objects = collision_world_->getCollisionObjects();
    int num_objects = collision_world_->getNumObjects();
    for (int i = 0; i < num_objects; /*i++*/)
    {
        auto world_obj = world_objects[i];
        // Make a marker
        visualization_msgs::msg::Marker mkr;
        mkr.ns = "collision_objects";
        mkr.header.frame_id = base_link_;
        mkr.action = visualization_msgs::msg::Marker::ADD;
        mkr.lifetime = rclcpp::Duration(0, 0);
        std::string obj_type = world_obj->object->getTypeString();

        // Get object pose relative to world_frame
        Eigen::Affine3d object_eig_pose = world_obj->object->object_transform;
        geometry_msgs::msg::Pose object_geo_pose;
        robot_collision_checking::fcl_interface::convertEigenTransformGeometryPose(object_eig_pose, object_geo_pose);
        mkr.pose = object_geo_pose;

        if (obj_type == "MESH")
        {
            geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.mesh), mkr);

            // Blue mesh
            mkr.id = world_obj->collision_id;
            mkr.color.b = 1.0;
            mkr.color.a = 1.0;
            marker_array_msg->markers.push_back(mkr);
            i++;
        }
        else if (obj_type == "PLANE")
        {
            mkr.scale.x = 10.0;
            mkr.scale.y = 10.0;
            mkr.scale.z = 0.001; // very thin

            // Red cuboid
            mkr.type = visualization_msgs::msg::Marker::CUBE;
            mkr.id = world_obj->collision_id;
            mkr.color.r = 1.0;
            mkr.color.a = 0.3;
            marker_array_msg->markers.push_back(mkr);
            i++;
        }
        else if (obj_type == "OCTOMAP")
        {
            // RCLCPP_WARN(this->get_logger(), "Unable to display octomap");
            i++;
        }
        else if (obj_type == "VOXEL_GRID")
        {
            mkr.id = world_obj->collision_id;
            // Treat voxel grid as a cube list
            mkr.type = visualization_msgs::msg::Marker::CUBE_LIST;
            auto grid = *(world_obj->object->ptr.voxel_grid);
            mkr.scale.x = grid.resolutions.x;
            mkr.scale.y = grid.resolutions.y;
            mkr.scale.z = grid.resolutions.z;
            // Voxel cells already account for position in world
            mkr.pose.position.x = mkr.pose.position.y = mkr.pose.position.z = 0.0;

            // Iterate over remaining cells until no more objects in world or a new collision object
            do
            {
                // The collision object is really a voxel cell
                auto voxel_cell = *(world_objects[i]->collision_object);
                fcl::Vector3d cell_center = voxel_cell.getTranslation();
                // Invert rotation to obtain cell position in original world frame
                Eigen::Matrix3d rotation_matrix = voxel_cell.getRotation().matrix();
                rotation_matrix.transposeInPlace();
                cell_center = rotation_matrix * cell_center;
                geometry_msgs::msg::Point point;
                point.x = cell_center[0];
                point.y = cell_center[1];
                point.z = cell_center[2];
                mkr.points.push_back(point);
                i++;
            } while ((i < num_objects) && (world_objects[i-1]->collision_id == world_objects[i]->collision_id));

            // Purple voxel grid
            mkr.color.r = 1.0;
            mkr.color.b = 1.0;
            mkr.color.a = 1.0;
            marker_array_msg->markers.push_back(mkr);
        }
        else if (obj_type == "SPHERE" || obj_type == "BOX" || obj_type == "CYLINDER" || obj_type == "CONE")
        {
            geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.solid), mkr);

            // Green primitives
            mkr.id = world_obj->collision_id;
            mkr.color.g = 1.0;
            mkr.color.a = 1.0;
            marker_array_msg->markers.push_back(mkr);
            i++;
        }
    }

    mkr_pub_->publish(*marker_array_msg);
}

void ConstrainedManipulability::convertCollisionModel(const GeometryInformation& geometry_information,
                                                      std::vector<shapes::ShapeMsg>& current_shapes,
                                                      std::vector<geometry_msgs::msg::Pose>& shapes_poses) const
{
    int num_shapes = geometry_information.shapes.size();
    current_shapes.resize(num_shapes);
    shapes_poses.resize(num_shapes);

    for (int i = 0; i < num_shapes; i++)
    {
        shapes::ShapeMsg current_shape;
        shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shapes[i]);
        
        shapes_poses[i] = tf2::toMsg(geometry_information.geometry_transforms[i]);
    }
}

void ConstrainedManipulability::createRobotCollisionModel(const GeometryInformation& geometry_information)
{
    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::msg::Pose> shapes_poses;
    convertCollisionModel(geometry_information, current_shapes, shapes_poses);

    int num_shapes = geometry_information.shapes.size();
    for (int i = 0; i < num_shapes; i++)
    {
        Eigen::Affine3d obj_pose;
        tf2::fromMsg(shapes_poses[i], obj_pose);
        if (current_shapes[i].which() == 0)
        {
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::SolidPrimitive>(current_shapes[i]), obj_pose);

            // Create the collision object
            robot_collision_checking::FCLCollisionGeometryPtr cg = robot_collision_checking::fcl_interface::createCollisionGeometry(obj);
            // Cache geometry of robot
            robot_collision_geometry_.push_back(cg);
        }
        else if (current_shapes[i].which() == 1)
        {
            robot_collision_checking::FCLObjectPtr obj = std::make_shared<robot_collision_checking::FCLObject>(
                boost::get<shape_msgs::msg::Mesh>(current_shapes[i]), robot_collision_checking::MESH, obj_pose);

            // Create the collision object
            robot_collision_checking::FCLCollisionGeometryPtr cg = robot_collision_checking::fcl_interface::createCollisionGeometry(obj);
            // Cache geometry of robot
            robot_collision_geometry_.push_back(cg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Collision geometry not supported");
        }
    }
}

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
        // Get current segment
        KDL::Segment seg = chain_.getSegment(i);
        // Get collision geometry
        urdf::CollisionSharedPtr link_coll = model_->links_.at(seg.getName())->collision;
        // If collision geometry does not exist at this link of the kinematic chain
        if (link_coll == nullptr)
        {
            Eigen::Affine3d base_T_ee;
            Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
            getKDLKinematicInformation(kdl_joint_positions, base_T_ee, base_J_ee);
            geometry_information.geometry_transforms.push_back(base_T_ee);
        }
        else
        {
            // Get collision geometry's shape
            std::unique_ptr<shapes::Shape> shape = constructShape(link_coll->geometry.get());
            // Get collision origin
            Eigen::Vector3d origin_Trans_collision(link_coll->origin.position.x, 
                                                   link_coll->origin.position.y, 
                                                   link_coll->origin.position.z);

            Eigen::Quaterniond origin_Quat_collision(link_coll->origin.rotation.w, 
                                                     link_coll->origin.rotation.x, 
                                                     link_coll->origin.rotation.y, 
                                                     link_coll->origin.rotation.z);

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
}

void ConstrainedManipulability::getJacobian(const sensor_msgs::msg::JointState& joint_state, Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    Eigen::Affine3d base_T_ee;
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, base_T_ee, Jac);
}

void ConstrainedManipulability::getTransform(const sensor_msgs::msg::JointState& joint_state, Eigen::Affine3d& T) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    Eigen::Matrix<double, 6, Eigen::Dynamic> Jac;
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);
    getKDLKinematicInformation(kdl_joint_positions, T, Jac);
}

void ConstrainedManipulability::getCartPos(const sensor_msgs::msg::JointState& joint_state, geometry_msgs::msg::Pose& geo_pose) const
{
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(chain_, joint_state, kdl_joint_positions);

    KDL::Frame cartpos;
    kdl_fk_solver_->JntToCart(kdl_joint_positions, cartpos);

    Eigen::Affine3d T;
    tf2::transformKDLToEigen(cartpos, T);
    geo_pose = tf2::toMsg(T);
}
} // namespace constrained_manipulability