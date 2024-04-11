
#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <geometry_msgs/msg/point.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>

#include "constrained_manipulability_interfaces/msg/polytope_mesh.hpp"

namespace constrained_manipulability
{
enum SLICING_PLANE
{
    YZ_PLANE = 0,
    XZ_PLANE = 1,
    XY_PLANE = 2,
};

class Polytope
{
    public:
        Polytope();
        Polytope(const std::string& name, const Eigen::MatrixXd& vertex_set);
        Polytope(const std::string& name, const Eigen::MatrixXd& A_left, const Eigen::VectorXd& b_left);
        ~Polytope() {}

        inline std::string getName() const { return name_; }
        inline double getVolume() const { return volume_; }

        // Compute the volume of a polytope defined by its vertex set
        double computeVolume() const;

        /** Get a Polytope's defined set of vertices
         *   The vertices are shifted to the offset position, e.g., the robot end effector
         *   PCL is used to obtain the convex hull
         *   A PolytopeMesh message is created from the convex hull and returned
         *   The return value is a flag for whether the computation was successful or not
         */
        bool getPolytopeMesh(const Eigen::Vector3d& offset,
                             std::vector<geometry_msgs::msg::Point>& points,
                             constrained_manipulability_interfaces::msg::PolytopeMesh& poly_mesh) const;

        /// Transform polytope to Cartesian space
        void transformCartesian(const Eigen::Matrix<double, 3, Eigen::Dynamic>& Jp);

        // yz plane = 0
        // xz plane = 1
        // xy plane = 2
        Polytope slice(const std::string& name,
                       SLICING_PLANE index = SLICING_PLANE::XY_PLANE,
                       double plane_width = 0.002) const;

        // Function to get the intersection of two polytopes
        Polytope getPolytopeIntersection(const std::string& name,
                                         const Eigen::MatrixXd& AHrep_other,
                                         const Eigen::VectorXd& bHrep_other) const;

    private:
        // Polytope properties
        std::string name_;
        double volume_;

        Eigen::MatrixXd vertex_set_;
        Eigen::MatrixXd AHrep_;
        Eigen::VectorXd bHrep_;
};
} // namespace constrained_manipulability