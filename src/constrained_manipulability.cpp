/*** INCLUDE FILES ***/
#include <constrained_manipulability/constrained_manipulability.h>

ConstrainedManipulability::ConstrainedManipulability(ros::NodeHandle nh,
                                                     std::string root,
                                                     std::string tip,
                                                     std::string robot_description,
                                                     double distance_threshold,
                                                     double linearization_limit,
                                                     double dangerfield) : nh_(nh), fclInterface(nh), distance_threshold_(distance_threshold), dangerfield_(dangerfield)
{

    octomap_subscriber = nh_.subscribe("constrained_manipulability/octomap_full", 1, &ConstrainedManipulability::octomapCallback, this);
    octomap_received_ = false;
    octomap_id_ = 99;
    octomap_pose_wrt_world_.setIdentity();

    polytope_server_ = nh_.advertiseService("get_polytope_constraints", &ConstrainedManipulability::getPolytopeConstraintsCallback, this);
    jacobian_server_ = nh_.advertiseService("get_jacobian_matrix", &ConstrainedManipulability::getJacobianCallback, this);

    wait_for_rviz = true;
    mkr_pub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    poly_mesh_pub = nh_.advertise<constrained_manipulability::PolytopeMesh>("constrained_manipulability/polytope_mesh", 1);
    obj_dist_pub = nh_.advertise<constrained_manipulability::ObjectDistances>("constrained_manipulability/obj_distances", 1);
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
                qmax_[mvable_jnt] = 2.0*M_PI;
                qmin_[mvable_jnt] = -2.0*M_PI;
            }
            else
            {
                qmax_[mvable_jnt] = model_.joints_.at(kdl_joint.getName())->limits->upper;
                qmin_[mvable_jnt] = model_.joints_.at(kdl_joint.getName())->limits->lower;
            }
            
            qdotmax_[mvable_jnt] = model_.joints_.at(kdl_joint.getName())->limits->velocity;
            qdotmin_[mvable_jnt] = -model_.joints_.at(kdl_joint.getName())->limits->velocity;
            joint_names[mvable_jnt]=kdl_joint.getName();
            mvable_jnt++;
        }
    }
    
    nh.setParam("constrained_manipulability/active_dof",joint_names);
    
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

    utility_functions::printVector(qmax_, "qmax_");
    utility_functions::printVector(qmin_, "qmin_");
    utility_functions::printVector(qdotmax_, "qdotmax_");
    utility_functions::printVector(qdotmin_, "qdotmin_");

    std::fill(max_lin_limit_.begin(), max_lin_limit_.end(), linearization_limit);
    std::fill(min_lin_limit_.begin(), min_lin_limit_.end(), -linearization_limit);

    ROS_INFO("Initialized");
}

void ConstrainedManipulability::setOctomapPose(const Eigen::Affine3d &wT1)
{
    octomap_pose_wrt_world_ = wT1;
}

void ConstrainedManipulability::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    octomap_ = *msg;
    octomap_received_ = true;

}

void ConstrainedManipulability::addOctomaptoWorld()
{
    // remove the old octomap from the world
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
        fclInterface.removeCollisionObject(octomap_id_);
        // update octomap from the world
        fclInterface.addCollisionObject(octomap_, octomap_pose_wrt_world_, octomap_id_);
    }
}

void ConstrainedManipulability::addFilteredOctomaptoWorld(const sensor_msgs::JointState &joint_states)
{
    KDL::JntArray kdl_joint_positions(ndof_);
    std::vector<shapes::ShapeMsg> current_shapes;
    std::vector<geometry_msgs::Pose> shapes_poses;
    GeometryInformation geometry_information;

    jointStatetoKDLJointArray(joint_states, kdl_joint_positions);    
    getCollisionModel(kdl_joint_positions, geometry_information);
    std::cout<<"get collision model scope"<<std::endl;
    convertCollisionModel(geometry_information,current_shapes,shapes_poses);
    std::cout<<"Pre scope"<<std::endl;
    // remove the old octomap from the world
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
        std::cout<<"Remove old collision object"<<std::endl;
        fclInterface.removeCollisionObject(octomap_id_);
        // update octomap from the world
        std::cout<<"Filtering"<<std::endl;                                             
        shape_msgs::Mesh s1=boost::get<shape_msgs::Mesh>(current_shapes[0]);                 
            std::cout<<"\n meshing complete \n \n \n"<<std::endl;
        
        std::cout<<"Stuck here"<<std::endl;
        // https://github.com/ros-planning/moveit_task_constructor/issues/241
        bodies::Body *body =bodies::constructBodyFromMsg ( s1,shapes_poses[0]);
        std::cout<<"Bodied it"<<std::endl;
                        
        FCLCollisionGeometryPtr cg=fclInterface.filterObjectFromOctomap(octomap_,
                                             current_shapes,
                                             shapes_poses);
        std::cout<<"Add new collision object"<<std::endl;                                             
        fclInterface.addCollisionObject(cg, octomap_pose_wrt_world_, octomap_id_);
    }
}

bool ConstrainedManipulability::convertCollisionModel(const GeometryInformation &geometry_information,
                            std::vector<shapes::ShapeMsg> & current_shapes,                                
                            std::vector<geometry_msgs::Pose> & shapes_poses)
{

    current_shapes.resize(geometry_information.geometry_transforms.size());
    shapes_poses.resize(geometry_information.geometry_transforms.size());

    for (int i = 0; i < geometry_information.geometry_transforms.size(); i++)
        {
        shapes::ShapeMsg current_shape;
        shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shapes[i]);
        tf::poseEigenToMsg(geometry_information.geometry_transforms[i], shapes_poses[i]);
        }
    return true;
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
        double vol(0.0);

        if (req.polytope_type == req.ALLOWABLE_MOTION_POLYTOPE)
        {
            vol = getAllowableMotionPolytope(req.sampled_joint_states[var], AHrep, bhrep, req.show_polytope);
        }
        else if (req.polytope_type == req.CONSTRAINED_ALLOWABLE_MOTION_POLYTOPE)
        {
            vol = getConstrainedAllowableMotionPolytope(req.sampled_joint_states[var], AHrep, bhrep, req.show_polytope);
        }
        else if (req.polytope_type == req.CONSTRAINED_VELOCITY_POLYTOPE)
        {
            vol = getConstrainedVelocityPolytope(req.sampled_joint_states[var], AHrep, bhrep, req.show_polytope);
        }
        else if (req.polytope_type == req.VELOCITY_POLYTOPE)
        {
            vol = getVelocityPolytope(req.sampled_joint_states[var], AHrep, bhrep, req.show_polytope);
        }
        else
        {
            ROS_ERROR("Unknown Polytope type");
            return false;
        }

        tf::matrixEigenToMsg(AHrep, res.polytope_hyperplanes[var].A);
        res.polytope_hyperplanes[var].b = utility_functions::eigenToVector(bhrep);
        res.polytope_hyperplanes[var].volume = vol;
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

void ConstrainedManipulability::setRvizWait(bool flag)
{
    wait_for_rviz = flag;
}

bool const ConstrainedManipulability::getRvizWait(bool flag)
{
    return wait_for_rviz;
}

bool ConstrainedManipulability::plotPolytope(std::string polytope_name,
                                             Eigen::MatrixXd vertices,
                                             std::string frame,
                                             Eigen::Vector3d position,
                                             std::vector<double> color_pts,
                                             std::vector<double> color_line)
{

    PointCloudPtr cloud_hull(new PointCloud);
    PointCloudPtr cloud_projected(new PointCloud);

    double vol(0.0);

    for (int var = 0; var < vertices.rows(); ++var)
    {
        pcl::PointXYZ p(vertices(var, 0) + position(0),
                        vertices(var, 1) + position(1),
                        vertices(var, 2) + position(2));
        cloud_projected->points.push_back(p);
    }

    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector<pcl::Vertices> polygons;
    try
    {
        chull.setInputCloud(cloud_projected);
        chull.reconstruct(*cloud_hull, polygons);
    }
    catch (...)
    {
        ROS_ERROR("ROS- plotPolytope: qhull error");
        return false;
    }

    if (!(cloud_hull->points.empty()))
    {

        std::vector<geometry_msgs::Point> points;
        points.clear();
        // points.resize(cloud_hull->points.size());

        constrained_manipulability::PolytopeMesh poly_mesh;
        poly_mesh.name = polytope_name;
        poly_mesh.mesh.triangles.resize(polygons.size());
        poly_mesh.mesh.vertices.resize(polygons.size() * 3);

        // Plotting
        visualization_msgs::Marker mkr;

        mkr.ns = polytope_name;
        mkr.action = visualization_msgs::Marker::ADD;
        mkr.type = visualization_msgs::Marker::TRIANGLE_LIST;
        mkr.header.frame_id = frame;

        // polygons is a vector of triangles represented by 3 indices
        // The indices correspond to points in cloud_hull
        // Therefore for each triangle in the polgyon
        // we find its three vertices and extract their x y z coordinates
        // this is then put in a
        for (int tri = 0; tri < polygons.size(); ++tri)
        {
            pcl::Vertices triangle = polygons[tri];

            for (int var = 0; var < 3; ++var)
            {
                geometry_msgs::Point pp;
                pp.x = cloud_hull->points[triangle.vertices[var]].x;
                pp.y = cloud_hull->points[triangle.vertices[var]].y;
                pp.z = cloud_hull->points[triangle.vertices[var]].z;
                points.push_back(pp);

                poly_mesh.mesh.triangles[tri].vertex_indices[var] = triangle.vertices[var];
                poly_mesh.mesh.vertices[triangle.vertices[var]] = pp;
            }
        }

        mkr.id = 2;
        mkr.lifetime = ros::Duration(0.0);
        mkr.color.r = color_line[0];
        mkr.color.g = color_line[1];
        mkr.color.b = color_line[2];
        mkr.color.a = color_line[3]; // fmax(auto_alpha,0.1);

        poly_mesh.color = mkr.color;

        mkr.scale.x = 1.0;
        mkr.scale.y = 1.0;
        mkr.scale.z = 1.0;
        mkr.points = points;
        while (mkr_pub.getNumSubscribers() < 1 && wait_for_rviz)
        {
            ROS_INFO("Waiting for subs");
            ros::spinOnce();
        }
        mkr_pub.publish(mkr);

        poly_mesh_pub.publish(poly_mesh);

        mkr.type = visualization_msgs::Marker::SPHERE_LIST;
        mkr.header.frame_id = frame;
        mkr.id = 1;
        mkr.lifetime = ros::Duration(0.0);
        mkr.color.r = color_pts[0];
        mkr.color.g = color_pts[1];
        mkr.color.b = color_pts[2];
        mkr.color.a = color_pts[3];
        mkr.scale.x = 0.005;
        mkr.scale.y = 0.005;
        mkr.scale.z = 0.005;
        mkr.points = points;

        while (mkr_pub.getNumSubscribers() < 1 && wait_for_rviz)
        {
            ROS_INFO("Waiting for subs");
            ros::spinOnce();
        }
        mkr_pub.publish(mkr);
    }
    else
    {
        ROS_WARN("plotPolytope: Hull empty");
        return false;
    }
    return true;
}

bool ConstrainedManipulability::displayMarker(visualization_msgs::Marker mkr,
                                              const Eigen::Affine3d &T,
                                              std::string frame,
                                              unsigned int obj_id,
                                              const Eigen::Vector4d &color)
{
    while (mkr_pub.getNumSubscribers() < 1 && wait_for_rviz)
    {
        ROS_INFO_ONCE("Waiting until marker is displayed in RVIZ");
        ros::Duration(0.05).sleep();
    }
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
    mkr_pub.publish(mkr);
    return true;
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

bool ConstrainedManipulability::removeCollisionObject(unsigned int object_id)
{
    boost::mutex::scoped_lock lock(collision_world_mutex_);
    return fclInterface.removeCollisionObject(object_id);
}
bool ConstrainedManipulability::addCollisionObject(FCLObjectSet objects)
{
    boost::mutex::scoped_lock lock(collision_world_mutex_);
    return fclInterface.addCollisionObject(objects);
}

bool ConstrainedManipulability::displayObjects()
{
    boost::mutex::scoped_lock lock(collision_world_mutex_);
    return fclInterface.displayObjects(base_link_);
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

    for (int i = 0; i < geometry_information.geometry_transforms.size(); i++)
    {
        boost::mutex::scoped_lock lock(collision_world_mutex_);
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

double ConstrainedManipulability::getPolytopeVolume(Eigen::MatrixXd vertices)
{
    PointCloudPtr cloud_hull(new PointCloud);
    PointCloudPtr cloud_projected(new PointCloud);
    double vol(0.0);
    // We are using PCL for the convex hull interface to qhull
    for (int var = 0; var < vertices.rows(); ++var)
    {
        pcl::PointXYZ p(vertices(var, 0),
                        vertices(var, 1),
                        vertices(var, 2));
        cloud_projected->points.push_back(p);
    }
    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector<pcl::Vertices> polygons;
    try
    {
        chull.setComputeAreaVolume(true);
        chull.setInputCloud(cloud_projected);
        chull.reconstruct(*cloud_hull, polygons);
    }
    catch (...)
    {
        ROS_ERROR("qhull error");
        return 0.0;
    }
    vol = chull.getTotalVolume();
    return vol;
}

double ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                             bool show_polytope,
                                                             std::vector<double> color_pts,
                                                             std::vector<double> color_line)
{

    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
    Eigen::MatrixXd Vset;
    Eigen::Vector3d offset_position;

    double vol = getAllowableMotionPolytope(joint_states,
                                            AHrep,
                                            bhrep,
                                            Vset,
                                            offset_position,
                                            show_polytope,
                                            color_pts,
                                            color_line);
    return vol;
}

double ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                             Eigen::MatrixXd &AHrep,
                                                             Eigen::VectorXd &bhrep,
                                                             bool show_polytope,
                                                             std::vector<double> color_pts,
                                                             std::vector<double> color_line)
{

    Eigen::MatrixXd Vset;
    Eigen::Vector3d offset_position;

    double vol = getAllowableMotionPolytope(joint_states,
                                            AHrep,
                                            bhrep,
                                            Vset,
                                            offset_position,
                                            show_polytope,
                                            color_pts,
                                            color_line);
    return vol;
}

double ConstrainedManipulability::getAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                             Eigen::MatrixXd &AHrep,
                                                             Eigen::VectorXd &bhrep,
                                                             Eigen::MatrixXd &Vset,
                                                             Eigen::Vector3d &offset_position,
                                                             bool show_polytope,
                                                             std::vector<double> color_pts,
                                                             std::vector<double> color_line)
{

    double vol_initial(-1);
    Eigen::Affine3d base_T_ee;
    Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
    Eigen::MatrixXd Qset;
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

    getVrepPolytope(AHrep,
                    bhrep,
                    Qset);

    getCartesianPolytope(Qset,
                         base_J_ee.topRows(3),
                         base_T_ee.translation(),
                         Vset);

    vol_initial = getPolytopeVolume(Vset);

    if (show_polytope)
    {
        plotPolytope("allowable_motion_polytope",
                     Vset,
                     base_link_,
                     base_T_ee.translation(),
                     color_pts,
                     color_line);
    }
    offset_position = base_T_ee.translation();
    return vol_initial;
}

double ConstrainedManipulability::getVelocityPolytope(const sensor_msgs::JointState &joint_states,
                                                      bool show_polytope,
                                                      std::vector<double> color_pts,
                                                      std::vector<double> color_line)
{
    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;

    double vol = getVelocityPolytope(joint_states,
                                     AHrep,
                                     bhrep,
                                     show_polytope,
                                     color_pts, color_line);
    return vol;
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

double ConstrainedManipulability::getVelocityPolytope(const sensor_msgs::JointState &joint_states,
                                                      Eigen::MatrixXd AHrep,
                                                      Eigen::VectorXd bhrep,
                                                      bool show_polytope,
                                                      std::vector<double> color_pts,
                                                      std::vector<double> color_line)
{

    double vol_initial(-1);
    KDL::JntArray kdl_joint_positions(ndof_);
    Eigen::Affine3d base_T_ee;
    Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_ee;
    Eigen::MatrixXd Qset, Vset;

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

    // Convert to V-representation
    getVrepPolytope(AHrep,
                    bhrep,
                    Qset);
    // Transform to Cartesian Space
    getCartesianPolytope(Qset,
                         base_J_ee.topRows(3),
                         base_T_ee.translation(),
                         Vset);
    // Calculate the volume
    vol_initial = getPolytopeVolume(Vset);

    // Display if desired
    if (show_polytope)
    {
        plotPolytope("velocity_polytope",
                     Vset,
                     base_link_,
                     base_T_ee.translation(),
                     color_pts,
                     color_line);
    }
    return vol_initial;
}

bool ConstrainedManipulability::slicePolytope(const Eigen::MatrixXd &Vset,
                                              Eigen::Vector3d offset_position,
                                              std::vector<double> color_pts,
                                              std::vector<double> color_line,
                                              std::string polytope_name,
                                              ConstrainedManipulability::SLICING_PLANE index,
                                              double plane_width)
{

    Eigen::IOFormat PythonFormat(4, 0, ", ", ",\n", "[", "]", "([", "])");
    Eigen::MatrixXd AHrep1, Vslice;
    Eigen::VectorXd bhrep1;

    // Get the H representation of the Cartesian polytope
    bool check_poly = getHrepPolytope(Vset,
                                      AHrep1,
                                      bhrep1);
    if (!check_poly)
    {
        return 0.0;
    }

    // Create the linear inequality constraints representing a plane with some width to avoid
    // dropping down in dimensions
    Eigen::MatrixXd AHrep2(6, AHrep1.cols()); // 6 is translational velocities
    Eigen::VectorXd bhrep2(6);
    bhrep2.setOnes();

    // this magic number is the other plane dimensions, we should be able to go as large as we want but it seems
    // to cause numerical issues meaning the display is an issue?
    // Anyway upper limit is display reasons
    // lower limit it must be bigger than polytope, so if you increase lin limit you need to increase this!!!
    bhrep2 = bhrep2 * plane_width * 100;
    AHrep2.setZero();

    for (int i = 0; i < AHrep2.rows(); i++)
    {
        for (int j = 0; j < AHrep2.cols(); j++)
        {
            if (i == j)
            {
                if (i < 3)
                {
                    AHrep2(i, j) = 1.0;
                    AHrep2(i + 3, j) = -1.0;
                }
            }
        }
    }

    // Set dimension orgonal to plane to a small number
    bhrep2[(int)index] = plane_width;
    bhrep2[((int)index) + 3] = plane_width;

    // Concacentate the polytope constraints to get the intersection
    bool valid_poly = getPolytopeIntersection(AHrep1,
                                              bhrep1,
                                              AHrep2,
                                              bhrep2,
                                              Vslice);

    if (!valid_poly)
    {
        return 0.0;
    }

    plotPolytope(polytope_name,
                 Vslice,
                 base_link_,
                 offset_position,
                 color_pts,
                 color_line);

    return valid_poly;
}

bool ConstrainedManipulability::getPolytopeIntersection(const Eigen::MatrixXd &AHrep1,
                                                        const Eigen::VectorXd &bhrep1,
                                                        const Eigen::MatrixXd &AHrep2,
                                                        const Eigen::VectorXd &bhrep2,
                                                        Eigen::MatrixXd &Vset)
{
    Eigen::MatrixXd AHrep3;
    Eigen::VectorXd bhrep3;

    //
    bool valid_poly = getPolytopeIntersection(AHrep1,
                                              bhrep1,
                                              AHrep2,
                                              bhrep2,
                                              AHrep3,
                                              bhrep3,
                                              Vset);
    return valid_poly;
}

bool ConstrainedManipulability::getPolytopeIntersection(const Eigen::MatrixXd &AHrep1,
                                                        const Eigen::VectorXd &bhrep1,
                                                        const Eigen::MatrixXd &AHrep2,
                                                        const Eigen::VectorXd &bhrep2,
                                                        Eigen::MatrixXd &AHrep3,
                                                        Eigen::VectorXd &bhrep3,
                                                        Eigen::MatrixXd &Vset)
{

    // To get intersection we simply stack the polytopes
    AHrep3.resize(AHrep1.rows() + AHrep2.rows(), AHrep1.cols());
    bhrep3.resize(bhrep1.rows() + bhrep2.rows());
    AHrep3.topRows(AHrep1.rows()) = AHrep1;
    AHrep3.bottomRows(AHrep2.rows()) = AHrep2;
    bhrep3.head(bhrep1.rows()) = bhrep1;
    bhrep3.tail(bhrep2.rows()) = bhrep2;
    // Convert to  v representation for plotting
    bool valid_poly = getVrepPolytope(AHrep3, bhrep3, Vset);

    return valid_poly;
}

double ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                        bool show_polytope,
                                                                        std::vector<double> color_pts,
                                                                        std::vector<double> color_line)
{

    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;
    Eigen::MatrixXd Vset;
    Eigen::Vector3d offset_position;

    double vol = getConstrainedAllowableMotionPolytope(joint_states,
                                                       AHrep,
                                                       bhrep,
                                                       Vset,
                                                       offset_position,
                                                       show_polytope,
                                                       color_pts,
                                                       color_line);

    return vol;
}

double ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                        Eigen::MatrixXd &AHrep,
                                                                        Eigen::VectorXd &bhrep,
                                                                        bool show_polytope,
                                                                        std::vector<double> color_pts,
                                                                        std::vector<double> color_line)
{

    Eigen::MatrixXd Vset;
    Eigen::Vector3d offset_position;

    double vol = getConstrainedAllowableMotionPolytope(joint_states,
                                                       AHrep,
                                                       bhrep,
                                                       Vset,
                                                       offset_position,
                                                       show_polytope,
                                                       color_pts,
                                                       color_line);
    return vol;
}

double ConstrainedManipulability::getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState &joint_states,
                                                                        Eigen::MatrixXd &AHrep,
                                                                        Eigen::VectorXd &bhrep,
                                                                        Eigen::MatrixXd &Vset,
                                                                        Eigen::Vector3d &offset_position,
                                                                        bool show_polytope,
                                                                        std::vector<double> color_pts,
                                                                        std::vector<double> color_line)
{

    Eigen::MatrixXd Qset;
    GeometryInformation geometry_information;
    double vol_reduced(0.0);
    KDL::JntArray kdl_joint_positions(ndof_);

    jointStatetoKDLJointArray(joint_states, kdl_joint_positions);
    getCollisionModel(kdl_joint_positions, geometry_information);

    bool collision_free = getPolytopeHyperPlanes(kdl_joint_positions,
                                                 geometry_information,
                                                 AHrep,
                                                 bhrep);

    if (!collision_free)
    {
        return 0.0;
    }

    bool valid_poly = getVrepPolytope(AHrep, bhrep, Qset);

    if (!valid_poly)
    {
        return 0.0;
    }
    getCartesianPolytope(Qset,
                         geometry_information.geometry_jacobians.back().topRows(3),
                         geometry_information.geometry_transforms.back().translation(),
                         Vset);
    if (show_polytope)
    {
        plotPolytope("constrained_allowable_motion_polytope",
                     Vset,
                     base_link_,
                     geometry_information.geometry_transforms.back().translation(),
                     color_pts,
                     color_line);
    }

    offset_position = geometry_information.geometry_transforms.back().translation();
    vol_reduced = getPolytopeVolume(Vset);
    return vol_reduced;
}

double ConstrainedManipulability::getConstrainedVelocityPolytope(const sensor_msgs::JointState &joint_states,
                                                                 bool show_polytope,
                                                                 std::vector<double> color_pts,
                                                                 std::vector<double> color_line)
{

    Eigen::MatrixXd AHrep;
    Eigen::VectorXd bhrep;

    double vol = getConstrainedVelocityPolytope(joint_states,
                                                AHrep, bhrep,
                                                show_polytope,
                                                color_pts,
                                                color_line);
    return vol;
}

double ConstrainedManipulability::getConstrainedVelocityPolytope(const sensor_msgs::JointState &joint_states,
                                                                 Eigen::MatrixXd &AHrep,
                                                                 Eigen::VectorXd &bhrep,
                                                                 bool show_polytope,
                                                                 std::vector<double> color_pts,
                                                                 std::vector<double> color_line)
{

    Eigen::MatrixXd Qset, Vset;
    GeometryInformation geometry_information;
    double vol_reduced(0.0);
    KDL::JntArray kdl_joint_positions(ndof_);
    jointStatetoKDLJointArray(joint_states, kdl_joint_positions);

    getCollisionModel(kdl_joint_positions, geometry_information);

    bool collision_free = getPolytopeHyperPlanes(kdl_joint_positions,
                                                 geometry_information,
                                                 AHrep,
                                                 bhrep, true);

    if (!collision_free)
    {
        return 0.0;
    }

    bool valid_poly = getVrepPolytope(AHrep, bhrep, Qset);

    if (!valid_poly)
    {
        return 0.0;
    }
    getCartesianPolytope(Qset,
                         geometry_information.geometry_jacobians.back().topRows(3),
                         geometry_information.geometry_transforms.back().translation(),
                         Vset);
    if (show_polytope)
    {
        plotPolytope("constrained_velocity_polytope",
                     Vset,
                     base_link_,
                     geometry_information.geometry_transforms.back().translation(),
                     color_pts,
                     color_line);
    }

    vol_reduced = getPolytopeVolume(Vset);
    return vol_reduced;
}

bool ConstrainedManipulability::getPolytopeHyperPlanes(
    const KDL::JntArray &kdl_joint_positions,
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
    obj_dist_pub.publish(dist_arr);

    return true;
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

bool ConstrainedManipulability::getCollisionModel(const KDL::JntArray &kdl_joint_positions,
                                                  GeometryInformation &geometry_information)
{

    geometry_information.clear();
    Eigen::Affine3d link_origin_T_collision_origin, base_T_link_origin, base_T_collision_origin;

    // Calculates the segement's collision geomtery
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
    return true;
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

bool ConstrainedManipulability::getVrepPolytope(const Eigen::MatrixXd &A_left,
                                                const Eigen::VectorXd &b_left,
                                                Eigen::MatrixXd &reduced_joint_vertex_set)
{

    Eigen::Polyhedron Poly;
    try
    {
        Poly.setHrep(A_left, b_left);
        auto vrep = Poly.vrep();
        reduced_joint_vertex_set = vrep.first;

        if (reduced_joint_vertex_set.rows() <= 0)
        {
            // ROS_ERROR ( "V representation error no rows" );
            return false;
        }
    }
    catch (...)
    {
        ROS_ERROR("V representation error");
        return false;
    }
    return true;
}

bool ConstrainedManipulability::getHrepPolytope(const Eigen::MatrixXd &vertex_set,
                                                Eigen::MatrixXd &A_left,
                                                Eigen::VectorXd &b_left)
{

    Eigen::Polyhedron Poly;
    Eigen::VectorXd v_type; // v_type indicates vertices or rays in polytope definition
    v_type.resize(vertex_set.rows());
    v_type.setOnes(); // Ones for vertices and zeros for rays

    try
    {
        Poly.setVertices(vertex_set);
        auto hrep = Poly.hrep();
        A_left = hrep.first;
        b_left = hrep.second;
    }
    catch (...)
    {
        ROS_ERROR("H representation error");
        return false;
    }
    return true;
}

void ConstrainedManipulability::getCartesianPolytope(Eigen::MatrixXd Q,
                                                     Eigen::Matrix<double, 3, Eigen::Dynamic> Jp,
                                                     Eigen::Vector3d P,
                                                     Eigen::MatrixXd &V)
{
    V.resize(Q.rows(), 3);
    V.setZero();
    transformVertexSet(Jp, Q, V);
}

void ConstrainedManipulability::transformVertexSet(Eigen::Matrix<double, 3, Eigen::Dynamic> J,
                                                   const Eigen::MatrixXd &vertex_set,
                                                   Eigen::MatrixXd &vertex_set_out)
{
    for (int var = 0; var < vertex_set.rows(); ++var)
    {
        Eigen::VectorXd vertex = vertex_set.row(var);
        Eigen::VectorXd p = J * vertex;
        vertex_set_out.row(var) = p;
    }
}

ConstrainedManipulability::~ConstrainedManipulability()
{
    ROS_INFO("Calling Destructor");
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
