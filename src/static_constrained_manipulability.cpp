/*** INCLUDE FILES ***/
#include <constrained_manipulability/constrained_manipulability.h>

// --------- STATIC FUNCTIONS FOR OPTIMIZATION-------------------------

void ConstrainedManipulability::jointStatetoKDLJointArray(KDL::Chain &chain, const sensor_msgs::JointState &joint_states,
                                                          KDL::JntArray &kdl_joint_positions)
{
    unsigned int jnt(0);
    for (int i = 0; i < chain.getNrOfSegments(); ++i)
    {

        KDL::Segment seg = chain.getSegment(i);
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

double ConstrainedManipulability::getConstrainedAllowableMotionPolytope(KDL::Chain &chain,
                                                                        urdf::Model &model,
                                                                        FCLObjectSet objects,
                                                                        const sensor_msgs::JointState &joint_states,
                                                                        Eigen::MatrixXd &AHrep,
                                                                        Eigen::VectorXd &bhrep,
                                                                        double linearization_limit,
                                                                        double distance_threshold)
{

    int ndof = chain.getNrOfJoints();

    Eigen::MatrixXd Qset, Vset;
    KDL::JntArray kdl_joint_positions(ndof);

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize(ndof, ndof);
    ndof_identity_matrix.setZero();
    for (int i = 0; i < ndof_identity_matrix.rows(); i++)
    {
        for (int j = 0; j < ndof_identity_matrix.cols(); j++)
        {
            if (i == j)
            {
                ndof_identity_matrix(i, j) = 1;
            }
        }
    }

    jointStatetoKDLJointArray(chain, joint_states, kdl_joint_positions);

    GeometryInformation geometry_information;

    getCollisionModel(chain, model, kdl_joint_positions, geometry_information);

    bool collision_free = getPolytopeHyperPlanes(chain,
                                                 model,
                                                 objects,
                                                 kdl_joint_positions,
                                                 geometry_information,
                                                 AHrep,
                                                 bhrep,
                                                 distance_threshold,
                                                 linearization_limit);

    if (!collision_free)
    {
        return 0.0;
    }

    bool valid_poly = getVrepPolytope(AHrep, bhrep, Qset);

    if (!valid_poly)
    {
        return 0.0;
    }
    getCartesianPolytope(Qset, geometry_information.geometry_jacobians.back().topRows(3), geometry_information.geometry_transforms.back().translation(), Vset);

    double vol_reduced = getPolytopeVolume(Vset);
    return vol_reduced;
}

double ConstrainedManipulability::getConstrainedVelocityPolytope(KDL::Chain &chain,
                                                                 urdf::Model &model,
                                                                 FCLObjectSet objects,
                                                                 const sensor_msgs::JointState &joint_states,
                                                                 Eigen::MatrixXd &AHrep,
                                                                 Eigen::VectorXd &bhrep,
                                                                 double dangerfield,
                                                                 double distance_threshold)
{

    int ndof = chain.getNrOfJoints();
    KDL::JntArray kdl_joint_positions(ndof);

    Eigen::MatrixXd Qset, Vset;

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize(ndof, ndof);
    ndof_identity_matrix.setZero();
    for (int i = 0; i < ndof_identity_matrix.rows(); i++)
    {
        for (int j = 0; j < ndof_identity_matrix.cols(); j++)
        {
            if (i == j)
            {
                ndof_identity_matrix(i, j) = 1;
            }
        }
    }

    jointStatetoKDLJointArray(chain, joint_states, kdl_joint_positions);

    GeometryInformation geometry_information;

    getCollisionModel(chain, model, kdl_joint_positions, geometry_information);

    bool collision_free = getPolytopeHyperPlanes(chain,
                                                 model,
                                                 objects,
                                                 kdl_joint_positions,
                                                 geometry_information,
                                                 AHrep,
                                                 bhrep,
                                                 distance_threshold,
                                                 0.0,
                                                 true, dangerfield);

    if (!collision_free)
    {
        return 0.0;
    }

    bool valid_poly = getVrepPolytope(AHrep, bhrep, Qset);

    if (!valid_poly)
    {
        return 0.0;
    }
    getCartesianPolytope(Qset, geometry_information.geometry_jacobians.back().topRows(3), geometry_information.geometry_transforms.back().translation(), Vset);

    double vol_reduced = getPolytopeVolume(Vset);
    return vol_reduced;
}

bool ConstrainedManipulability::getPolytopeHyperPlanes(KDL::Chain &chain,
                                                       urdf::Model &model,
                                                       FCLObjectSet objects,
                                                       const KDL::JntArray &kdl_joint_positions,
                                                       const GeometryInformation &geometry_information,
                                                       Eigen::MatrixXd &AHrep,
                                                       Eigen::VectorXd &bhrep,
                                                       double distance_threshold,
                                                       double linearization_limit,
                                                       bool velocity_polytope,
                                                       double dangerfield)
{

    int ndof = chain.getNrOfJoints();

    std::vector<double> qmax(ndof), qmin(ndof), qdotmax(ndof), qdotmin(ndof);

    int mvable_jnt(0);
    //for ( int i=0; i<ndof+1; ++i ) {
    for (int i = 0; i < chain.getNrOfSegments(); ++i)
    {
        KDL::Segment seg = chain.getSegment(i);
        KDL::Joint kdl_joint = seg.getJoint();

        if (kdl_joint.getType() != KDL::Joint::None)
        {
            qmax[mvable_jnt] = model.joints_.at(kdl_joint.getName())->limits->upper;
            qmin[mvable_jnt] = model.joints_.at(kdl_joint.getName())->limits->lower;

            qdotmax[mvable_jnt] = model.joints_.at(kdl_joint.getName())->limits->velocity;
            qdotmin[mvable_jnt] = -model.joints_.at(kdl_joint.getName())->limits->velocity;

            mvable_jnt++;
        }
    }

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize(ndof, ndof);
    ndof_identity_matrix.setZero();
    for (int i = 0; i < ndof_identity_matrix.rows(); i++)
    {
        for (int j = 0; j < ndof_identity_matrix.cols(); j++)
        {
            if (i == j)
            {
                ndof_identity_matrix(i, j) = 1;
            }
        }
    }
    // Object ids
    std::vector<int> obj_ids;
    // Closest points
    std::vector<Eigen::Vector3d> p1w, p2w;
    // Min distance to object
    std::vector<double> obj_distances;
    Eigen::Vector3d nt;
    std::vector<double> distances;
    distances.clear();
    std::vector<Eigen::Matrix<double, 1, Eigen::Dynamic>> J_constraints;
    J_constraints.clear();
    // For all robot links
    for (int i = 0; i < geometry_information.shapes.size(); i++)
    {

        shapes::ShapeMsg current_shape;
        shapes::constructMsgFromShape(geometry_information.shapes[i].get(), current_shape);

        if (current_shape.which() == 0)
        {
            FCLInterface::checkDistanceObjectWorld(boost::get<shape_msgs::SolidPrimitive>(current_shape),
                                                   geometry_information.geometry_transforms[i],
                                                   objects,
                                                   obj_distances,
                                                   p1w,
                                                   p2w);
        }
        else if (current_shape.which() == 1)
        {
            FCLInterface::checkDistanceObjectWorld(boost::get<shape_msgs::Mesh>(current_shape),
                                                   geometry_information.geometry_transforms[i],
                                                   objects,
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
            Eigen::Matrix<double, 6, Eigen::Dynamic> w_J_out_p1; //
            Eigen::Matrix<double, 1, Eigen::Dynamic> J_proj;
            J_proj.setZero();
            w_J_out_p1.setZero();

            if (obj_distances[j] < 0.0)
            {
                // in collision
                ROS_WARN(" In collision");
                return false;
            }
            else if (obj_distances[j] < distance_threshold)
            {

                // Save the distance
                Eigen::Vector3d rdiff = p2w[j] - p1w[j];
                nt = rdiff; // direction of obstacle
                nt.normalize();
                // Get Jacobian at link
                Eigen::Vector3d w_delta_p1_collision_origin = p1w[j] - geometry_information.geometry_transforms[i].translation();
                // Get the Jacobian at p1

                screwTransform(geometry_information.geometry_jacobians[i],
                               w_J_out_p1,
                               w_delta_p1_collision_origin);
                projectTranslationalJacobian(nt, w_J_out_p1, J_proj);
                J_constraints.push_back(J_proj);
                if (velocity_polytope)
                {
                    distances.push_back(dangerfield * (obj_distances[j] * obj_distances[j]) - obj_distances[j]);
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
    AHrep.resize(offset * ndof + J_constraints.size(),
                 ndof);
    bhrep.resize(offset * ndof + distances.size(), 1);
    AHrep.setZero();
    bhrep.setZero();

    if (velocity_polytope)
    {                                                             // If velocity , then the joint position constraints simply scale max velocity
        AHrep.topRows(ndof) = ndof_identity_matrix;               // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block(ndof, 0, ndof, ndof) = -ndof_identity_matrix; // ndof_*ndof block at row  ndof_ colum 0;ndof_

        for (int i = 0; i < ndof; ++i)
        {
            double qmean = (qmax[i] + qmin[i]) / 2;
            double val_max = fmax(qmean, kdl_joint_positions(i)) - qmean;
            double val_min = fmin(qmean, kdl_joint_positions(i)) - qmean;
            double dmax = pow((((val_max) / ((qmax[i] - qmean)))), 2);
            double dmin = pow((((val_min) / ((qmin[i] - qmean)))), 2);
            // Make sure the value is witin joint limits and these limits are correctly defined.
            ROS_ASSERT(~std::isnan(dmax) && ~std::isnan(dmin) && ~std::isinf(dmax) && ~std::isinf(dmin));
            bhrep(i) = (1 - dmax) * qdotmax[i];
            bhrep(i + ndof) = (1 - dmin) * -qdotmin[i];
        }
    }
    else
    {

        AHrep.topRows(ndof) = ndof_identity_matrix;                   // ndof_*ndof block at row  0 colum 0;ndof_
        AHrep.block(ndof, 0, ndof, ndof) = ndof_identity_matrix;      // ndof_*ndof block at row  ndof_ colum 0;ndof_
        AHrep.block(ndof * 2, 0, ndof, ndof) = ndof_identity_matrix;  // ndof_*ndof block at row  ndof_*2 colum 0;ndof_
        AHrep.block(ndof * 3, 0, ndof, ndof) = -ndof_identity_matrix; // ndof_*ndof block at row  ndof_*3 colum 0;ndof_ -ndof_identity_matrix_ for negative joint limits
        for (int i = 0; i < ndof; ++i)
        {
            bhrep(i) = qmax[i] - kdl_joint_positions(i);
            bhrep(i + ndof) = kdl_joint_positions(i) - qmin[i];
            bhrep(i + 2 * ndof) = linearization_limit;
            bhrep(i + 3 * ndof) = linearization_limit;
        }
    }

    for (int var = 0; var < J_constraints.size(); ++var)
    {
        AHrep.row(offset * ndof + var) = J_constraints[var];
        bhrep(offset * ndof + var) = distances[var]; // Small tolerance to stop passing through
    }
    return true;
}

bool ConstrainedManipulability::getCollisionModel(KDL::Chain &chain,
                                                  urdf::Model &model,
                                                  const KDL::JntArray &kdl_joint_positions,
                                                  GeometryInformation &geometry_information)
{

    geometry_information.clear();

    KDL::ChainJntToJacSolver kdl_dfk_solver(chain);
    KDL::ChainFkSolverPos_recursive kdl_fk_solver(chain);

    Eigen::Affine3d link_origin_T_collision_origin, base_T_link_origin, base_T_collision_origin;

    int ndof = chain.getNrOfJoints();

    // Calculates the segement's collision geomtery
    //  The transform to the origin of the collision geometry
    //  The Jacobian matrix at the origin of the collisiNon geometry
    for (int i = 0; i < chain.getNrOfSegments(); ++i)
    {
        KDL::Segment seg = chain.getSegment(i); // Get current segment

        // Get Collision Geometry
        // Get Collision Geometry
        std::unique_ptr<shapes::Shape> shape = constructShape(model.links_.at(seg.getName())->collision->geometry.get());

        // Get Collision Origin
        Eigen::Vector3d origin_Trans_collision(model.links_.at(seg.getName())->collision->origin.position.x,
                                               model.links_.at(seg.getName())->collision->origin.position.y,
                                               model.links_.at(seg.getName())->collision->origin.position.z);
        Eigen::Quaterniond origin_Quat_collision(
            model.links_.at(seg.getName())->collision->origin.rotation.w,
            model.links_.at(seg.getName())->collision->origin.rotation.x,
            model.links_.at(seg.getName())->collision->origin.rotation.y,
            model.links_.at(seg.getName())->collision->origin.rotation.z);

        link_origin_T_collision_origin.translation() = origin_Trans_collision;
        link_origin_T_collision_origin.linear() = origin_Quat_collision.toRotationMatrix();

        // Finds cartesian pose w.r.t to base frame
        Eigen::Matrix<double, 6, Eigen::Dynamic> base_J_collision_origin;
        KDL::Frame cartpos;
        kdl_fk_solver.JntToCart(kdl_joint_positions, cartpos, i + 1);

        tf::transformKDLToEigen(cartpos, base_T_link_origin);

        base_T_collision_origin = base_T_link_origin * link_origin_T_collision_origin;

        // Get Jacobian at collision geometry origin
        KDL::Jacobian base_J_link_origin;
        base_J_link_origin.resize(ndof);

        kdl_dfk_solver.JntToJac(kdl_joint_positions, base_J_link_origin, i + 1);
        Eigen::MatrixXd Jac = base_J_link_origin.data;

        base_T_collision_origin = base_T_link_origin * link_origin_T_collision_origin;
        Eigen::Vector3d base_L_link_collision = (base_T_link_origin.linear() * link_origin_T_collision_origin.translation());
        screwTransform(base_J_link_origin.data, base_J_collision_origin, base_L_link_collision);

        // Push back solutions
        geometry_information.shapes.push_back(std::move(shape));
        geometry_information.geometry_transforms.push_back(base_T_collision_origin);
        geometry_information.geometry_jacobians.push_back(base_J_collision_origin);
    }
    return true;
}

double ConstrainedManipulability::getAllowableMotionPolytope(KDL::Chain &chain,
                                                             urdf::Model &model,
                                                             const sensor_msgs::JointState &joint_states,
                                                             Eigen::MatrixXd &Ahrep_undeformed,
                                                             Eigen::VectorXd &bhrep_underformed,
                                                             double linearization_limit)
{

    int ndof = chain.getNrOfJoints();
    KDL::JntArray kdl_joint_positions(ndof);
    KDL::ChainJntToJacSolver kdl_dfk_solver(chain);
    KDL::ChainFkSolverPos_recursive kdl_fk_solver(chain);

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize(ndof, ndof);
    ndof_identity_matrix.setZero();
    for (int i = 0; i < ndof_identity_matrix.rows(); i++)
    {
        for (int j = 0; j < ndof_identity_matrix.cols(); j++)
        {
            if (i == j)
            {
                ndof_identity_matrix(i, j) = 1;
            }
        }
    }

    jointStatetoKDLJointArray(chain, joint_states, kdl_joint_positions);

    std::vector<double> max_lin_limit(ndof);
    std::vector<double> min_lin_limit(ndof);

    std::fill(max_lin_limit.begin(), max_lin_limit.end(), linearization_limit);
    std::fill(min_lin_limit.begin(), min_lin_limit.end(), -linearization_limit);

    Eigen::Affine3d base_T_ee;
    KDL::Frame cartpos;
    KDL::Jacobian base_J_link_origin;
    base_J_link_origin.resize(ndof);
    kdl_fk_solver.JntToCart(kdl_joint_positions, cartpos);
    tf::transformKDLToEigen(cartpos, base_T_ee);
    kdl_dfk_solver.JntToJac(kdl_joint_positions, base_J_link_origin);
    Eigen::MatrixXd base_J_ee = base_J_link_origin.data;

    //     Eigen::MatrixXd Ahrep_undeformed;
    //     Eigen::VectorXd bhrep_underformed;
    Eigen::MatrixXd Qset_undeformed, Vset_undeformed;
    Ahrep_undeformed.resize(2 * ndof, ndof);
    Ahrep_undeformed.topRows(ndof) = ndof_identity_matrix;               // ndof_*ndof block at row  0 colum 0;ndof_
    Ahrep_undeformed.block(ndof, 0, ndof, ndof) = -ndof_identity_matrix; // ndof_*ndof block at row  ndof_ colum 0;ndof_
    bhrep_underformed.resize(2 * ndof, 1);
    for (int i = 0; i < ndof; ++i)
    {
        bhrep_underformed(i) = max_lin_limit[i];
        bhrep_underformed(i + ndof) = -min_lin_limit[i];
    }

    getVrepPolytope(Ahrep_undeformed, bhrep_underformed, Qset_undeformed);
    getCartesianPolytope(Qset_undeformed, base_J_ee.topRows(3), base_T_ee.translation(), Vset_undeformed);
    double vol_initial = getPolytopeVolume(Vset_undeformed);
    return vol_initial;
}

double ConstrainedManipulability::getVelocityPolytope(KDL::Chain &chain,
                                                      urdf::Model &model,
                                                      const sensor_msgs::JointState &joint_states,
                                                      Eigen::MatrixXd &AHrep,
                                                      Eigen::VectorXd &bhrep)
{

    int ndof = chain.getNrOfJoints();
    KDL::JntArray kdl_joint_positions(ndof);
    KDL::ChainJntToJacSolver kdl_dfk_solver(chain);
    KDL::ChainFkSolverPos_recursive kdl_fk_solver(chain);

    Eigen::MatrixXd ndof_identity_matrix;
    ndof_identity_matrix.resize(ndof, ndof);
    ndof_identity_matrix.setZero();
    for (int i = 0; i < ndof_identity_matrix.rows(); i++)
    {
        for (int j = 0; j < ndof_identity_matrix.cols(); j++)
        {
            if (i == j)
            {
                ndof_identity_matrix(i, j) = 1;
            }
        }
    }

    std::vector<double> qdot_max(ndof);
    std::vector<double> qdot_min(ndof);

    int mvable_jnt(0);
    //for ( int i=0; i<ndof+1; ++i ) {
    for (int i = 0; i < chain.getNrOfSegments(); ++i)
    {
        KDL::Segment seg = chain.getSegment(i);
        KDL::Joint kdl_joint = seg.getJoint();

        if (kdl_joint.getType() != KDL::Joint::None)
        {
            qdot_max[mvable_jnt] = model.joints_.at(kdl_joint.getName())->limits->velocity;
            qdot_min[mvable_jnt] = -model.joints_.at(kdl_joint.getName())->limits->velocity;
            mvable_jnt++;
        }
    }

    jointStatetoKDLJointArray(chain, joint_states, kdl_joint_positions);

    std::vector<double> max_lin_limit(ndof);
    std::vector<double> min_lin_limit(ndof);

    Eigen::Affine3d base_T_ee;
    KDL::Frame cartpos;
    KDL::Jacobian base_J_link_origin;
    base_J_link_origin.resize(ndof);
    kdl_fk_solver.JntToCart(kdl_joint_positions, cartpos);
    tf::transformKDLToEigen(cartpos, base_T_ee);
    kdl_dfk_solver.JntToJac(kdl_joint_positions, base_J_link_origin);
    Eigen::MatrixXd base_J_ee = base_J_link_origin.data;

    Eigen::MatrixXd Ahrep_undeformed;
    Eigen::VectorXd bhrep_underformed;
    Eigen::MatrixXd Qset_undeformed, Vset_undeformed;
    Ahrep_undeformed.resize(2 * ndof, ndof);
    Ahrep_undeformed.topRows(ndof) = ndof_identity_matrix;               // ndof_*ndof block at row  0 colum 0;ndof_
    Ahrep_undeformed.block(ndof, 0, ndof, ndof) = -ndof_identity_matrix; // ndof_*ndof block at row  ndof_ colum 0;ndof_
    bhrep_underformed.resize(2 * ndof, 1);
    for (int i = 0; i < ndof; ++i)
    {
        bhrep_underformed(i) = qdot_max[i];
        bhrep_underformed(i + ndof) = -qdot_min[i];
    }

    getVrepPolytope(Ahrep_undeformed, bhrep_underformed, Qset_undeformed);
    getCartesianPolytope(Qset_undeformed, base_J_ee.topRows(3), base_T_ee.translation(), Vset_undeformed);
    double vol_initial = getPolytopeVolume(Vset_undeformed);
    return vol_initial;
}
