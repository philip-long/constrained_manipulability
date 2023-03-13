
#ifndef POLYTOPE_HPP
#define POLYTOPE_HPP

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <geometry_msgs/Point.h>

#include <constrained_manipulability/PolytopeMesh.h>

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
            Polytope(std::string name, const Eigen::MatrixXd &vertex_set);
            Polytope(std::string name, const Eigen::MatrixXd &A_left, const Eigen::VectorXd &b_left);
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
            bool getPolytopeMesh(const Eigen::Vector3d &offset,
                    std::vector<geometry_msgs::Point> &points,
                    constrained_manipulability::PolytopeMesh &poly_mesh) const;

            /// Transform polytope to Cartesian space
            void transformCartesian(Eigen::Matrix<double, 3, Eigen::Dynamic> Jp, Eigen::Vector3d P);

            // yz plane = 0
            // xz plane = 1
            // xy plane = 2
            Polytope slice(std::string name,
                        constrained_manipulability::SLICING_PLANE index = constrained_manipulability::SLICING_PLANE::XY_PLANE,
                        double plane_width = 0.002) const;

            // Function to get the intersection of two polytopes
            Polytope getPolytopeIntersection(std::string name,
                                            const Eigen::MatrixXd &AHrep_other,
                                            const Eigen::VectorXd &bHrep_other) const;

        private:
            // Polytope properties
            std::string name_;
            double volume_;

            Eigen::MatrixXd vertex_set_;
            Eigen::MatrixXd AHrep_;
            Eigen::VectorXd bHrep_;
    };
}

#endif