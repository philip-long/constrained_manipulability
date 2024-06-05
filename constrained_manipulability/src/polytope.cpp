#include <eigen-cddlib/Polyhedron.h>

#include <pcl/point_types.h>
#include <pcl/surface/impl/convex_hull.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "constrained_manipulability/polytope.hpp"

namespace constrained_manipulability
{
namespace{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

rclcpp::Logger getLogger()
{
    return rclcpp::get_logger("constrained_manipulability.polytope");
}
} // namespace

Polytope::Polytope() : name_("invalid_polytope"), volume_(0.0)
{
}

Polytope::Polytope(const std::string& polytope_name, const Eigen::MatrixXd& vertex_set, const Eigen::Vector3d& offset) 
    : name_(polytope_name), vertex_set_(vertex_set)
{
    // Convert from V rep to H rep
    Eigen::Polyhedron poly;
    Eigen::VectorXd v_type; // v_type indicates vertices or rays in polytope definition
    v_type.resize(vertex_set.rows());
    v_type.setOnes(); // Ones for vertices and zeros for rays

    try
    {
        poly.setVertices(vertex_set);
        auto hrep = poly.hrep();
        AHrep_ = hrep.first;
        bHrep_ = hrep.second;

        volume_ = computeVolume();

        if (!constructMesh(offset))
        {
            RCLCPP_ERROR(getLogger(), "Failed to construct mesh");
        }
    }
    catch (...)
    {
        RCLCPP_ERROR(getLogger(), "H representation error");
    }
}

Polytope::Polytope(const std::string& polytope_name, const Eigen::MatrixXd& A_left, const Eigen::VectorXd& b_left, const Eigen::Vector3d& offset)
    : name_(polytope_name), AHrep_(A_left), bHrep_(b_left)
{
    // Convert from H rep to V rep
    Eigen::Polyhedron poly;
    try
    {
        poly.setHrep(A_left, b_left);
        auto vrep = poly.vrep();
        vertex_set_ = vrep.first;
        
        volume_ = computeVolume();
        
        if (vertex_set_.rows() <= 0)
        {
            // RCLCPP_INFO(getLogger(), "V representation has no rows");
        }
        else 
        {   
            // If V rep has rows try constructing mesh
            if (!constructMesh(offset))
            {
                RCLCPP_ERROR(getLogger(), "Failed to construct mesh");
            }
        }
    }
    catch (...)
    {
        RCLCPP_ERROR(getLogger(), "V representation error");
    }
}

double Polytope::computeVolume() const
{
    if (name_ == "invalid_polytope")
    {
        return 0.0;
    }

    PointCloudPtr cloud_hull(new PointCloud);
    PointCloudPtr cloud_projected(new PointCloud);

    // Use PCL for the convex hull interface to qhull
    for (int var = 0; var < vertex_set_.rows(); ++var)
    {
        pcl::PointXYZ p(vertex_set_(var, 0),
                        vertex_set_(var, 1),
                        vertex_set_(var, 2));
        cloud_projected->points.push_back(p);
    }

    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector<pcl::Vertices> polygons;
    try
    {
        chull.setComputeAreaVolume(false);
        chull.setInputCloud(cloud_projected);
        chull.reconstruct(*cloud_hull, polygons);
    }
    catch (...)
    {
        RCLCPP_ERROR(getLogger(), "Qhull error");
        return 0.0;
    }

    return chull.getTotalVolume();
}

bool Polytope::constructMesh(const Eigen::Vector3d& offset)
{
    if (name_ == "invalid_polytope")
    {
        return false;
    }

    PointCloudPtr cloud_hull(new PointCloud);
    PointCloudPtr cloud_projected(new PointCloud);

    for (int var = 0; var < vertex_set_.rows(); ++var)
    {
        pcl::PointXYZ p(vertex_set_(var, 0) + offset(0),
                        vertex_set_(var, 1) + offset(1),
                        vertex_set_(var, 2) + offset(2));
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
        // RCLCPP_ERROR(getLogger(), "plot: Qhull error");
        return false;
    }

    if (!(cloud_hull->points.empty()))
    {
        points_.clear();

        unsigned int n_triangles = polygons.size();
        unsigned int n_vertices = cloud_hull->points.size();

        // Generating the polytope mesh
        for (unsigned int i = 0; i < n_vertices; i++)
        {
            geometry_msgs::msg::Point pp;
            pp.x = cloud_hull->points[i].x;
            pp.y = cloud_hull->points[i].y;
            pp.z = cloud_hull->points[i].z;
            mesh_.vertices.push_back(pp);
        }

        for (unsigned int i = 0; i < n_triangles; i++)
        {
            shape_msgs::msg::MeshTriangle tri;
            tri.vertex_indices[0] = polygons[i].vertices[0];
            tri.vertex_indices[1] = polygons[i].vertices[1];
            tri.vertex_indices[2] = polygons[i].vertices[2];
            mesh_.triangles.push_back(tri);
        }

        // Also producing a vector of points for the visualization marker
        // Polygons are a vector of triangles represented by 3 indices
        // The indices correspond to points in cloud_hull
        // Therefore for each triangle in the polygon
        // we find its three vertices and extract their x y z coordinates
        for (unsigned int tri = 0; tri < n_triangles; ++tri)
        {
            pcl::Vertices triangle = polygons[tri];

            for (int var = 0; var < 3; ++var)
            {
                geometry_msgs::msg::Point pp;
                pp.x = cloud_hull->points[triangle.vertices[var]].x;
                pp.y = cloud_hull->points[triangle.vertices[var]].y;
                pp.z = cloud_hull->points[triangle.vertices[var]].z;
                points_.push_back(pp);
            }
        }
    }
    else
    {
        // RCLCPP_WARN(getLogger(), "plot: Qhull empty");
        return false;
    }

    return true;
}

void Polytope::transformCartesian(const Eigen::Matrix<double, 3, Eigen::Dynamic>& Jp, const Eigen::Vector3d& offset)
{
    if (name_ == "invalid_polytope")
    {
        return;
    }

    Eigen::MatrixXd V;
    V.resize(vertex_set_.rows(), 3);
    V.setZero();

    for (int var = 0; var < vertex_set_.rows(); ++var)
    {
        Eigen::VectorXd vertex = vertex_set_.row(var);
        Eigen::VectorXd p = Jp * vertex;
        V.row(var) = p;
    }

    // Deep copy
    vertex_set_ = V;

    // Changing state, so make sure to re-compute volume and mesh
    volume_ = computeVolume();
    constructMesh(offset);
}

Polytope Polytope::slice(const std::string& name, SLICING_PLANE index, double plane_width) const
{
    if (name_ == "invalid_polytope")
    {
        return Polytope();
    }

    Eigen::IOFormat PythonFormat(4, 0, ", ", ",\n", "[", "]", "([", "])");

    // Create the linear inequality constraints representing a plane with some width to avoid
    // dropping down in dimensions
    Eigen::MatrixXd AHrep_other(6, AHrep_.cols()); // 6 is translational velocities
    Eigen::VectorXd bHrep_other(6);
    bHrep_other.setOnes();

    // this magic number is the other plane dimensions, we should be able to go as large as we want but it seems
    // to cause numerical issues meaning the display is an issue?
    // Anyway upper limit is display reasons
    // lower limit it must be bigger than polytope, so if you increase lin limit you need to increase this!!!
    bHrep_other = bHrep_other * plane_width * 100;
    AHrep_other.setZero();

    for (int i = 0; i < AHrep_other.rows(); i++)
    {
        for (int j = 0; j < AHrep_other.cols(); j++)
        {
            if (i == j)
            {
                if (i < 3)
                {
                    AHrep_other(i, j) = 1.0;
                    AHrep_other(i + 3, j) = -1.0;
                }
            }
        }
    }

    // Set dimension orgonal to plane to a small number
    bHrep_other[static_cast<int> (index)] = plane_width;
    bHrep_other[(static_cast<int> (index)) + 3] = plane_width;

    // Concacentate the polytope constraints to get the intersection
    return getPolytopeIntersection(name, AHrep_other, bHrep_other);
}

Polytope Polytope::getPolytopeIntersection(const std::string& name, 
                                           const Eigen::MatrixXd& AHrep_other, 
                                           const Eigen::VectorXd& bHrep_other) const
{
    if (name_ == "invalid_polytope")
    {
        return Polytope();
    }

    Eigen::MatrixXd AHrep_out;
    Eigen::VectorXd bHrep_out;

    // To get intersection we simply stack the polytopes
    AHrep_out.resize(AHrep_.rows() + AHrep_other.rows(), AHrep_.cols());
    bHrep_out.resize(bHrep_.rows() + bHrep_other.rows());
    AHrep_out.topRows(AHrep_.rows()) = AHrep_;
    AHrep_out.bottomRows(AHrep_other.rows()) = AHrep_other;
    bHrep_out.head(bHrep_.rows()) = bHrep_;
    bHrep_out.tail(bHrep_other.rows()) = bHrep_other;

    return Polytope(name, AHrep_out, bHrep_out);
}
} // namespace constrained_manipulability