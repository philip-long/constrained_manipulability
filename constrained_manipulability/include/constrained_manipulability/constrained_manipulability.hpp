#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>

// TODO: later versions of ROS 2, e.g., Humble, require tf2_eigen.hpp
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <urdf/model.h>

#include <rclcpp/rclcpp.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "robot_collision_checking/fcl_interface.hpp"

#include "constrained_manipulability/constrained_manipulability_utils.hpp"
#include "constrained_manipulability/polytope.hpp"

#include "constrained_manipulability_interfaces/msg/object_distances.hpp"
#include "constrained_manipulability_interfaces/msg/polytope_mesh.hpp"
#include "constrained_manipulability_interfaces/msg/polytope_volume.hpp"
#include "octomap_filter_interfaces/msg/filter_mesh.hpp"

#include "constrained_manipulability_interfaces/srv/get_polytope_constraints.hpp"
#include "constrained_manipulability_interfaces/srv/get_jacobian_matrix.hpp"

namespace constrained_manipulability
{
/// GeometryInformation contains all geomertic information obtained from chain
struct GeometryInformation
{
    std::vector<std::unique_ptr<shapes::Shape>> shapes;
    TransformVector geometry_transforms;
    JacobianVector geometry_jacobians;

    void clear()
    {
        shapes.clear();
        geometry_transforms.clear();
        geometry_jacobians.clear();
    }
};

class ConstrainedManipulability : public rclcpp::Node
{
    public:
        ConstrainedManipulability();
        ~ConstrainedManipulability(){};

    //     /// Add a solid primitive object to FCLInterface collision world transform w.r.t base_link of chain
    //     bool addCollisionObject(const shape_msgs::SolidPrimitive& s1,
    //                             const Eigen::Affine3d& wT1, unsigned int object_id);
    //     /// Add a mesh object to FCLInterface collision world transform w.r.t base_link of chain
    //     bool addCollisionObject(const shape_msgs::Mesh& s1,
    //                             const Eigen::Affine3d& wT1, unsigned int object_id);
    //     /// Add an Octomap map object to FCLInterface collision world transform w.r.t base_link of chain
    //     bool addCollisionObject(const octomap_msgs::Octomap& map,
    //                             const Eigen::Affine3d& wT1, unsigned int object_id);
    //     /// Add a set of object to FCLInterface collision world transform w.r.t base_link of chain
    //     bool addCollisionObject(robot_collision_checking::FCLObjectSet objects);
    //     /// Remove an object from FCLInterface collision world
    //     bool removeCollisionObject(unsigned int object_id);

    //     /// Display collision world objects
    //     bool displayObjects();
    //     /// Display calculated collision model in rviz
    //     bool displayCollisionModel(sensor_msgs::JointState const& joint_states);
    //     /// Display a marker, in a given reference frame defined by a visualization_msgs::Marker
    //     bool displayMarker(visualization_msgs::Marker mkr,
    //                     const Eigen::Affine3d& T,
    //                     std::string frame,
    //                     unsigned int obj_id,
    //                     const Eigen::Vector4d& color);
    //     /// Checks the current state for collision
    //     bool checkCollision(const sensor_msgs::JointState& joint_states);

    //     /// Convenience function to get end-effector pose as a geometry_msgs::Pose
    //     void getCartPos(const sensor_msgs::JointState& joint_states,
    //                     geometry_msgs::Pose& geo_pose);

    //     /** getAllowableMotionPolytope returns the polytope
    //      *   considering     linearization
    //      *
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the allowable motion polytope
    //      */
    //     Polytope getAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
    //                                         bool show_polytope,
    //                                         std::vector<double> color_pts = {0.0, 0.0, 0.5, 0.0},
    //                                         std::vector<double> color_line = {0.0, 0.0, 1.0, 0.4});

    //     /** getAllowableMotionPolytope returns the polytope
    //      *   considering   linearization
    //      *
    //      *  AHrep hyperplanes of the allowable motion polytope
    //      *  bHrep shifted distance
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the allowable motion polytope
    //      */
    //     Polytope getAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
    //                                         Eigen::MatrixXd& AHrep,
    //                                         Eigen::VectorXd& bhrep,
    //                                         bool show_polytope,
    //                                         std::vector<double> color_pts = {0.0, 0.0, 0.5, 0.0},
    //                                         std::vector<double> color_line = {0.0, 0.0, 1.0, 0.4});

    //     /** getAllowableMotionPolytope returns the polytope
    //      *   considering   linearization
    //      *
    //      *  AHrep hyperplanes of the allowable motion polytope
    //      *  bHrep shifted distance
    //      *  Eigen::MatrixXd&  Vset, the cartesian V representation of polytope
    //      *  Eigen::Vector3d&  offset_position the position we translate the polytopte to
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the allowable motion polytope
    //      */
    //     Polytope getAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
    //                                         Eigen::MatrixXd& AHrep,
    //                                         Eigen::VectorXd& bhrep,
    //                                         Eigen::Vector3d& offset_position,
    //                                         bool show_polytope,
    //                                         std::vector<double> color_pts,
    //                                         std::vector<double> color_line);

    //     /** getConstrainedAllowableMotionPolytope returns the polytope
    //      *   approximating the constrained allowable end effector motion, considering
    //      *  joint limits and objstacles&  linearization
    //      *
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the constrained allowable end effector motion polytope
    //      */
    //     Polytope getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
    //                                                     bool show_polytope,
    //                                                     std::vector<double> color_pts = {0.0, 0.0, 0.5, 0.0},
    //                                                     std::vector<double> color_line = {1.0, 0.0, 0.0, 0.4});

    //     /** getConstrainedAllowableMotionPolytope returns the polytope
    //      *   approximating the constrained allowable end effector motion, considering
    //      *  joint limits and objstacles&  linearization
    //      *
    //      *  AHrep hyperplanes of the constrained allowable motion polytope
    //      *  bHrep shifted distance
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the constrained allowable motion polytope
    //      */
    //     Polytope getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
    //                                                     Eigen::MatrixXd& AHrep,
    //                                                     Eigen::VectorXd& bhrep,
    //                                                     bool show_polytope,
    //                                                     std::vector<double> color_pts = {0.0, 0.0, 0.5, 0.0},
    //                                                     std::vector<double> color_line = {1.0, 0.0, 0.0, 0.4});

    //     /** getConstrainedAllowableMotionPolytope returns the polytope
    //      *   approximating the constrained allowable end effector motion, considering
    //      *  joint limits and objstacles&  linearization
    //      *
    //      *  AHrep hyperplanes of the constrained allowable motion polytope
    //      *  bHrep shifted distance
    //      *  offset_position the location in space of the Cartesian polytope
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the constrained allowable motion polytope
    //      */
    //     Polytope getConstrainedAllowableMotionPolytope(const sensor_msgs::JointState& joint_states,
    //                                                     Eigen::MatrixXd& AHrep,
    //                                                     Eigen::VectorXd& bhrep,
    //                                                     Eigen::Vector3d& offset_position,
    //                                                     bool show_polytope,
    //                                                     std::vector<double> color_pts,
    //                                                     std::vector<double> color_line);

    //     /** getVelocityPolytope returns the manipulability polytope
    //      *   considering joint velocity limits
    //      *
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the manipulability polytope
    //      */
    //     Polytope getVelocityPolytope(const sensor_msgs::JointState& joint_states,
    //                                     bool show_polytope,
    //                                     std::vector<double> color_pts = {0.0, 0.5, 0.0, 1.0},
    //                                     std::vector<double> color_line = {0.0, 1.0, 0.0, 0.8});

    //     /** getVelocityManipulabilityPolytope returns the manipulability polytope
    //     *   considering joint velocity limits

    //     *  AHrep hyperplanes of the constrained allowable motion polytope
    //     *  bHrep shifted distance
    //     *  const sensor_msgs::JointState&  joint_states, current joint states
    //     *  show_polytope -> plots the polytope in rviz
    //     *  color_pts -> polytope points color
    //     *  color_line  -> polytope lines color
    //     *  returns the manipulability polytope
    //     */
    //     Polytope getVelocityPolytope(const sensor_msgs::JointState& joint_states,
    //                                     Eigen::MatrixXd AHrep,
    //                                     Eigen::VectorXd bhrep,
    //                                     bool show_polytope,
    //                                     std::vector<double> color_pts = {0.0, 0.5, 0.0, 1.0},
    //                                     std::vector<double> color_line = {0.0, 1.0, 0.0, 0.8});

    //     /** getConstrainedVelocityPolytope returns the polytope
    //      *   approximating the constrained allowable end effector velocities, considering
    //      *  joint velocity limits and objtacles and dangerfield values
    //      *
    //      *  AHrep hyperplanes of the constrained allowable motion polytope
    //      *  bHrep shifted distance
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the constrained velocity motion polytope
    //      */
    //     Polytope getConstrainedVelocityPolytope(const sensor_msgs::JointState& joint_states,
    //                                             bool show_polytope,
    //                                             std::vector<double> color_pts = {0.0, 0.5, 0.0, 1.0},
    //                                             std::vector<double> color_line = {0.0, 1.0, 0.0, 0.8});

    //     /** getConstrainedVelocityPolytope returns the polytope
    //      *   approximating the constrained allowable end effector velocities, considering
    //      *  joint velocity limits and objtacles and dangerfield values
    //      *
    //      *  AHrep hyperplanes of the constrained allowable motion polytope
    //      *  bHrep shifted distance
    //      *  const sensor_msgs::JointState&  joint_states, current joint states
    //      *  show_polytope -> plots the polytope in rviz
    //      *  color_pts -> polytope points color
    //      *  color_line  -> polytope lines color
    //      *  returns the constrained velocity motion polytope
    //      */
    //     Polytope getConstrainedVelocityPolytope(const sensor_msgs::JointState& joint_states,
    //                                             Eigen::MatrixXd& AHrep,
    //                                             Eigen::VectorXd& bhrep,
    //                                             bool show_polytope,
    //                                             std::vector<double> color_pts = {0.0, 0.5, 0.0, 1.0},
    //                                             std::vector<double> color_line = {0.0, 1.0, 0.0, 0.8});

    //     /** getPolytopeHyperPlanes returns hyperplanes for constrained joint polytope
    //      * For ith link (segment) in the kinematic serial chain, we return
    //      *  AHrep is the normal to he half spaces
    //      *  bhrep constains the shifted distance from the origin along the normal
    //      * Polytope is then defined as P= { A x <= b }
    //      * velocity_polytope true means returns hyperplanes based on velocity and dangerfield
    //      *                   false means the free spoace approximation is returned.
    //      */
    //     bool getPolytopeHyperPlanes(
    //         const KDL::JntArray& jointpositions,
    //         GeometryInformation& geometry_information,
    //         Eigen::MatrixXd& AHrep,
    //         Eigen::VectorXd& bhrep,
    //         bool velocity_polytope = false);

    //     /** Plot a Polytope defined by a set of vertices
    //      *   The facet color is defined by color_line, while points by color_pts
    //      */
    //     bool plotPolytope(const Polytope& poly,
    //                         const Eigen::Vector3d& offset_position,
    //                         std::vector<double> color_pts = {1.0, 0.4, 0.4, 1.0},
    //                         std::vector<double> color_line = {1.0, 0.4, 0.4, 1.0}) const;

        // Getter for Jacobian based on a given joint state
        void getJacobian(const sensor_msgs::msg::JointState& joint_states, Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac) const;

        // Getter for transform matrix based on a given joint state
        void getTransform(const sensor_msgs::msg::JointState& joint_states, Eigen::Affine3d& T) const;
    
        // Set a new limit for all joints
        inline void setLinearizationLimit(double linearization_limit)
        {
            std::fill(max_lin_limit_.begin(), max_lin_limit_.end(), linearization_limit);
            std::fill(min_lin_limit_.begin(), min_lin_limit_.end(), -linearization_limit);
        }

        // Set a linearization limit for one joint
        inline void setLinearizationLimit(double linearization_limit, unsigned int joint)
        {
            max_lin_limit_[joint] = linearization_limit;
            min_lin_limit_[joint] = linearization_limit;
        }

        // Getter for current linearization limit, which is the same for all joints
        inline double getLinearizationLimit() const
        {
            return max_lin_limit_[0];
        }

    private:
        // This function is taken from https://github.com/tu-darmstadt-ros-pkg with a tiny change to smart ptrs
        // https://github.com/tu-darmstadt-ros-pkg/robot_self_filter/blob/master/src/self_mask.cpp#L76
        std::unique_ptr<shapes::Shape> constructShape(const urdf::Geometry* geom) const;

        // Convenience function to convert KDL to Eigen, segment=-1 returns terminal point information
        void getKDLKinematicInformation(const KDL::JntArray& kdl_joint_positions, Eigen::Affine3d& T,
                                        Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac, int segment = -1) const;

        // getCollisionModel returns  kinematic information about collision geometry
        // For ith link (segment) in the kinematic serial chain, we return
        // GeometryInformation:
        //      shapes[i]: the shape description of geometery [i]
        //      geometry_transforms[i]: the transform of the collision geometry's [i] origin
        //      geometric_jacobians[i]: the jacobian matrix in the base frame at the collision geometry's [i] origin
        void getCollisionModel(const KDL::JntArray& joint_positions, GeometryInformation& geometry_information) const;

        void convertCollisionModel(const GeometryInformation& geometry_information,
                                   std::vector<shapes::ShapeMsg>& current_shapes,
                                   std::vector<geometry_msgs::msg::Pose>& shapes_poses) const;
        
        // ROS service callbacks
        void getJacobianCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetJacobianMatrix::Request> req,
                                 std::shared_ptr<constrained_manipulability_interfaces::srv::GetJacobianMatrix::Response> res);
        void getPolytopeConstraintsCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetPolytopeConstraints::Request> req,
                                            std::shared_ptr<constrained_manipulability_interfaces::srv::GetPolytopeConstraints::Response> res);

        /// Constrained manipulability data members
        // Robot's base link and end-effector frame
        std::string base_link_;
        std::string tip_;
        // Distance threshold beyond which objects are ignored
        double distance_threshold_;
        // Desired dangerfield value
        double dangerfield_;
        // Linearization limits
        std::vector<double> max_lin_limit_;
        std::vector<double> min_lin_limit_;
        /// Joint limits
        std::vector<double> qmax_;
        std::vector<double> qmin_;
        std::vector<double> qdotmax_;
        std::vector<double> qdotmin_;
        // Robot's number of DoF
        unsigned int ndof_;
        Eigen::MatrixXd ndof_identity_matrix_;

        // Collision checking
        std::shared_ptr<robot_collision_checking::FCLInterfaceCollisionWorld> collision_world_;
        boost::mutex collision_world_mutex_;
        
        // Robot kinematics
        KDL::Chain chain_;
        std::shared_ptr<urdf::Model> model_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> kdl_dfk_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;

        // Tracking the octomap ID if available
        int octomap_id_;

        // ROS services
        rclcpp::Service<constrained_manipulability_interfaces::srv::GetJacobianMatrix>::SharedPtr jacobian_server_;
        rclcpp::Service<constrained_manipulability_interfaces::srv::GetPolytopeConstraints>::SharedPtr polytope_server_;

        // ROS publishers
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mkr_pub_;
        rclcpp::Publisher<constrained_manipulability_interfaces::msg::ObjectDistances>::SharedPtr obj_dist_pub_;
        rclcpp::Publisher<constrained_manipulability_interfaces::msg::PolytopeMesh>::SharedPtr poly_mesh_pub_;
        rclcpp::Publisher<constrained_manipulability_interfaces::msg::PolytopeVolume>::SharedPtr poly_vol_pub_;
        rclcpp::Publisher<octomap_filter_interfaces::msg::FilterMesh>::SharedPtr obj_filt_pub_;
};
} // namespace constrained_manipulability