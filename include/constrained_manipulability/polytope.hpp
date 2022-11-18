
#ifndef POLYTOPE_HPP
#define POLYTOPE_HPP

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <ros/ros.h>

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

        Polytope(ros::NodeHandle nh, std::string name, std::string frame, const Eigen::MatrixXd &vertex_set);
        Polytope(ros::NodeHandle nh, std::string name, std::string frame, const Eigen::MatrixXd &A_left, const Eigen::VectorXd &b_left);
        ~Polytope() {}

        // Get the volume of a polytope defined by its vertex set
        double getVolume() const;

        /** Plot a Polytope defined by a set of vertices
         *   The vertices are shifted to the offset position, e.g. the robot end effector
         *   PCL is used to obtain the convex hull
         *   A mesh message is created from the convex hull and published to RViz
         *   The facet color is defined by color_line, while points by color_pts
         *   The return value is the volume of the polytope
         */
        bool plot(Eigen::Vector3d offset_position,
                  std::vector<double> color_pts = {1.0, 0.4, 0.4, 1.0},
                  std::vector<double> color_line = {1.0, 0.4, 0.4, 1.0}) const;

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
        ros::NodeHandle nh_;

        /// RViz Display publishers
        ros::Publisher mkr_pub_;
        ros::Publisher poly_mesh_pub_;
        ros::Publisher poly_vol_pub_;

        // Polytope properties
        std::string frame_;
        std::string name_;
        Eigen::MatrixXd vertex_set_;
        Eigen::MatrixXd AHrep_;
        Eigen::VectorXd bHrep_;
        double volume_;
    };
}

#endif