#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "robot_collision_checking/fcl_interface.hpp"

#include "constrained_manipulability/constrained_manipulability_utils.hpp"
#include "constrained_manipulability/polytope.hpp"

#include "octomap_filter_interfaces/msg/filter_mesh.hpp"
#include "octomap_filter_interfaces/msg/filter_primitive.hpp"

#include "constrained_manipulability_interfaces/msg/matrix.hpp"
#include "constrained_manipulability_interfaces/msg/object_distances.hpp"
#include "constrained_manipulability_interfaces/msg/polytope.hpp"

#include "constrained_manipulability_interfaces/srv/add_remove_collision_mesh.hpp"
#include "constrained_manipulability_interfaces/srv/add_remove_collision_solid.hpp"
#include "constrained_manipulability_interfaces/srv/get_jacobian_matrix.hpp"
#include "constrained_manipulability_interfaces/srv/get_polytopes.hpp"
#include "constrained_manipulability_interfaces/srv/get_sliced_polytope.hpp"

namespace constrained_manipulability
{
/// GeometryInformation contains all geometric information obtained from chain
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

const int OCTOMAP_ID = 777;

class ConstrainedManipulability : public rclcpp::Node
{
    public:
        explicit ConstrainedManipulability(const rclcpp::NodeOptions& options);
        ~ConstrainedManipulability(){};

    private:
        /// Constrained manipulability ROS interface

        // ROS service/subscriber/timer callbacks
        void addRemoveMeshCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh::Request> req,
                                   std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh::Response> res);
        void addRemoveSolidCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid::Request> req,
                                    std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid::Response> res);
        void getJacobianCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetJacobianMatrix::Request> req,
                                 std::shared_ptr<constrained_manipulability_interfaces::srv::GetJacobianMatrix::Response> res);
        void getPolytopesCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetPolytopes::Request> req,
                                  std::shared_ptr<constrained_manipulability_interfaces::srv::GetPolytopes::Response> res);
        void getSlicedPolytopeCallback(const std::shared_ptr<constrained_manipulability_interfaces::srv::GetSlicedPolytope::Request> req,
                                       std::shared_ptr<constrained_manipulability_interfaces::srv::GetSlicedPolytope::Response> res);

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
        void linLimitCallback(const std_msgs::msg::Float32::SharedPtr msg);

        void checkCollisionCallback();
        void polytopePubCallback();

        /// Constrained manipulability private methods

        // Add an FCLObject to FCLInterface collision world transform w.r.t base_link of chain
        bool addCollisionObject(const robot_collision_checking::FCLObjectPtr& obj, int object_id);

        // Remove an object from FCLInterface collision world
        bool removeCollisionObject(int object_id);

        /** getPolytopeHyperPlanes returns hyperplanes for a constrained joint polytope
         * 
         *  For the ith link (segment) in the kinematic serial chain, we return:
         *  AHrep -> the normal to the half spaces
         *  bHrep -> the shifted distance from the origin along the normal
         * 
         *  The polytope is then defined as P = { Ax <= b }
         *  velocity_polytope true means the hyperplanes are return based on velocity and dangerfield values
         *                    false means the free spoace approximation is returned
         */
        bool getPolytopeHyperPlanes(const KDL::JntArray& jointpositions,
                                    const GeometryInformation& geometry_information,
                                    const builtin_interfaces::msg::Time& joint_state_stamp,
                                    Eigen::MatrixXd& AHrep,
                                    Eigen::VectorXd& bHrep,
                                    bool velocity_polytope = false);

        /** getAllowableMotionPolytope returns the polytope considering linearization
         *
         *  joint_state -> a given joint state
         *  show_polytope -> plots the polytope in RViz
         *  color_pts -> polytope points color
         *  color_line -> polytope lines color
         *  AHrep hyperplanes of the allowable motion polytope
         *  bHrep shifted distance
         *  Eigen::Vector3d offset_position the location in space of the Cartesian polytope
         *  returns the allowable motion polytope
         */
        Polytope getAllowableMotionPolytope(const sensor_msgs::msg::JointState& joint_state,
                                            bool show_polytope,
                                            Eigen::MatrixXd& AHrep,
                                            Eigen::VectorXd& bHrep,
                                            Eigen::Vector3d& offset_position,
                                            const std::vector<double>& color_pts = {0.0, 0.0, 0.5, 0.0},
                                            const std::vector<double>& color_line = {0.0, 0.0, 1.0, 0.4}) const;
        // Function overload if output parameters not required
        Polytope getAllowableMotionPolytope(const sensor_msgs::msg::JointState& joint_state,
                                            bool show_polytope,
                                            const std::vector<double>& color_pts = {0.0, 0.0, 0.5, 0.0},
                                            const std::vector<double>& color_line = {0.0, 0.0, 1.0, 0.4}) const;

        /** getConstrainedAllowableMotionPolytope returns the polytope
         *  that approximates the constrained allowable end effector motion, 
         *  considering joint limits, obstacles, and linearization
         *
         *  joint_state -> a given joint state
         *  show_polytope -> plots the polytope in RViz
         *  AHrep hyperplanes of the constrained allowable motion polytope
         *  bHrep shifted distance
         *  Eigen::Vector3d offset_position the location in space of the Cartesian polytope
         *  returns the constrained allowable motion polytope
         *  color_pts -> polytope points color
         *  color_line -> polytope lines color
         */
        Polytope getConstrainedAllowableMotionPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                       bool show_polytope, 
                                                       Eigen::MatrixXd& AHrep,
                                                       Eigen::VectorXd& bHrep,
                                                       Eigen::Vector3d& offset_position,
                                                       const std::vector<double>& color_pts = {0.0, 0.0, 0.5, 0.0}, 
                                                       const std::vector<double>& color_line = {1.0, 0.0, 0.0, 0.4});
        // Function overload if output parameters not required
        Polytope getConstrainedAllowableMotionPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                       bool show_polytope, 
                                                       const std::vector<double>& color_pts = {0.0, 0.0, 0.5, 0.0}, 
                                                       const std::vector<double>& color_line = {1.0, 0.0, 0.0, 0.4});

        /** getVelocityPolytope returns the manipulability polytope considering joint velocity limits
         * 
         *  joint_state -> a given joint state
         *  show_polytope -> plots the polytope in RViz
         *  AHrep hyperplanes of the velocity polytope
         *  bHrep shifted distance
         *  Eigen::Vector3d offset_position the location in space of the Cartesian polytope
         *  returns the allowable velocity polytope
         *  color_pts -> polytope points color
         *  color_line -> polytope lines color
        */
        Polytope getVelocityPolytope(const sensor_msgs::msg::JointState& joint_state,
                                     bool show_polytope,
                                     Eigen::MatrixXd& AHrep,
                                     Eigen::VectorXd& bHrep,
                                     Eigen::Vector3d& offset_position,
                                     const std::vector<double>& color_pts = {0.0, 0.5, 0.0, 1.0},
                                     const std::vector<double>& color_line = {0.0, 1.0, 0.0, 0.8}) const;
        // Function overload if output parameters not required
        Polytope getVelocityPolytope(const sensor_msgs::msg::JointState& joint_state,
                                     bool show_polytope,
                                     const std::vector<double>& color_pts = {0.0, 0.5, 0.0, 1.0},
                                     const std::vector<double>& color_line = {0.0, 1.0, 0.0, 0.8}) const;

        /** getConstrainedVelocityPolytope returns the polytope
         *  that approximates the constrained allowable end effector velocities, 
         *  considering joint velocity limits, obstacles, and dangerfield values
         *
         *  joint_state -> a given joint state
         *  show_polytope -> plots the polytope in RViz
         *  AHrep hyperplanes of the velocity polytope
         *  bHrep shifted distance
         *  Eigen::Vector3d offset_position the location in space of the Cartesian polytope
         *  returns the consetrained velocity motion polytope
         *  color_pts -> polytope points color
         *  color_line -> polytope lines color
         */
        Polytope getConstrainedVelocityPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                bool show_polytope,
                                                Eigen::MatrixXd& AHrep,
                                                Eigen::VectorXd& bHrep,
                                                Eigen::Vector3d& offset_position,
                                                const std::vector<double>& color_pts = {0.0, 0.5, 0.0, 1.0},
                                                const std::vector<double>& color_line = {0.0, 1.0, 0.0, 0.8});
        // Function overload if output parameters not required
        Polytope getConstrainedVelocityPolytope(const sensor_msgs::msg::JointState& joint_state,
                                                bool show_polytope,
                                                const std::vector<double>& color_pts = {0.0, 0.5, 0.0, 1.0},
                                                const std::vector<double>& color_line = {0.0, 1.0, 0.0, 0.8});

        /** Plot a Polytope defined by a set of vertices
         *  The facet color is defined by color_line, while points by color_pts
         */
        void plotPolytope(const std::string& poly_name,
                          const std::vector<geometry_msgs::msg::Point>& points,
                          const std::vector<double>& color_pts = {1.0, 0.4, 0.4, 1.0}, 
                          const std::vector<double>& color_line = {1.0, 0.4, 0.4, 1.0}) const;

        // Check for collisions given a joint state
        bool checkCollision(const sensor_msgs::msg::JointState& joint_state);

        // Display calculated collision model in RViz
        void displayCollisionModel(const GeometryInformation& geometry_information, const Eigen::Vector4d& color = {0.1, 0.5, 0.2, 0.5}) const;

        void convertCollisionModel(const GeometryInformation& geometry_information,
                                   std::vector<shapes::ShapeMsg>& current_shapes,
                                   std::vector<geometry_msgs::msg::Pose>& shapes_poses) const;

        // Create robot collision geometry after receiving first robot geometry information instance
        void createRobotCollisionModel(const GeometryInformation& geometry_information);

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

        // Getter for Jacobian based on a given joint state
        void getJacobian(const sensor_msgs::msg::JointState& joint_state, Eigen::Matrix<double, 6, Eigen::Dynamic>& Jac) const;

        // Getter for transform matrix based on a given joint state
        void getTransform(const sensor_msgs::msg::JointState& joint_state, Eigen::Affine3d& T) const;
    
        // Convenience function to get end-effector pose as a geometry_msgs::Pose
        void getCartPos(const sensor_msgs::msg::JointState& joint_state, geometry_msgs::msg::Pose& geo_pose) const;

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

        // Bool properties whether to publish manipulability/velocity polytopes or not
        bool publish_mp_;
        bool publish_cmp_;
        bool publish_vp_;
        bool publish_cvp_;

        // Bool properties whether to show manipulability/velocity polytopes or not
        bool show_mp_;
        bool show_cmp_;
        bool show_vp_;
        bool show_cvp_;

        // Collision checking
        std::shared_ptr<robot_collision_checking::FCLInterfaceCollisionWorld> collision_world_;
        boost::mutex collision_world_mutex_;
        std::vector<robot_collision_checking::FCLCollisionGeometryPtr> robot_collision_geometry_;

        // Robot kinematics
        KDL::Chain chain_;
        std::unique_ptr<urdf::Model> model_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> kdl_dfk_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;

        sensor_msgs::msg::JointState joint_state_;
        boost::mutex joint_state_mutex_;

        // Transforms to correctly filter objects from octomap (irrespective of frame differences)
        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> listener_;

        // ROS services
        rclcpp::Service<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh>::SharedPtr mesh_coll_server_;
        rclcpp::Service<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid>::SharedPtr solid_coll_server_;
        rclcpp::Service<constrained_manipulability_interfaces::srv::GetJacobianMatrix>::SharedPtr jacobian_server_;
        rclcpp::Service<constrained_manipulability_interfaces::srv::GetPolytopes>::SharedPtr polytopes_server_;
        rclcpp::Service<constrained_manipulability_interfaces::srv::GetSlicedPolytope>::SharedPtr sliced_polytope_server_;

        // ROS subscribers/publishers/timers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_filter_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lin_limit_sub_;

        rclcpp::TimerBase::SharedPtr coll_check_timer_;
        rclcpp::TimerBase::SharedPtr polytope_pub_timer_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mkr_pub_;
        rclcpp::Publisher<constrained_manipulability_interfaces::msg::ObjectDistances>::SharedPtr obj_dist_pub_;
        rclcpp::Publisher<constrained_manipulability_interfaces::msg::Polytope>::SharedPtr poly_pub_;
        rclcpp::Publisher<octomap_filter_interfaces::msg::FilterMesh>::SharedPtr filt_mesh_pub_;
        rclcpp::Publisher<octomap_filter_interfaces::msg::FilterPrimitive>::SharedPtr filt_prim_pub_;
};
} // namespace constrained_manipulability