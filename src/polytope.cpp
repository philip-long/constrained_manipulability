#include <eigen-cddlib/Polyhedron.h>

#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>

#include <ros/ros.h>

#include <constrained_manipulability/polytope.hpp>

namespace constrained_manipulability
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;

    Polytope::Polytope() : name_("invalid_polytope"), volume_(0.0)
    {
    }

    Polytope::Polytope(std::string polytope_name, const Eigen::MatrixXd &vertex_set)
        : name_(polytope_name), vertex_set_(vertex_set)
    {
        // Convert from V rep to H rep
        Eigen::Polyhedron Poly;
        Eigen::VectorXd v_type; // v_type indicates vertices or rays in polytope definition
        v_type.resize(vertex_set.rows());
        v_type.setOnes(); // Ones for vertices and zeros for rays

        try
        {
            Poly.setVertices(vertex_set);
            auto hrep = Poly.hrep();
            AHrep_ = hrep.first;
            bHrep_ = hrep.second;

            volume_ = computeVolume();
        }
        catch (...)
        {
            ROS_ERROR("H representation error");
        }
    }

    Polytope::Polytope(std::string polytope_name, const Eigen::MatrixXd &A_left, const Eigen::VectorXd &b_left)
        : name_(polytope_name), AHrep_(A_left), bHrep_(b_left)
    {
        // Convert from H rep to V rep
        Eigen::Polyhedron Poly;
        try
        {
            Poly.setHrep(A_left, b_left);
            auto vrep = Poly.vrep();
            vertex_set_ = vrep.first;

            volume_ = computeVolume();

            if (vertex_set_.rows() <= 0)
            {
                // ROS_INFO("V representation has no rows");
            }
        }
        catch (...)
        {
            ROS_ERROR("V representation error");
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
            chull.setComputeAreaVolume(true);
            chull.setInputCloud(cloud_projected);
            chull.reconstruct(*cloud_hull, polygons);
        }
        catch (...)
        {
            ROS_ERROR("qhull error");
            return 0.0;
        }

        return chull.getTotalVolume();
    }

    bool Polytope::getPolytopeMesh(const Eigen::Vector3d &offset,
                                   std::vector<geometry_msgs::Point> &points,
                                   constrained_manipulability::PolytopeMesh &poly_mesh) const
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
            ROS_ERROR("plot: qhull error");
            return false;
        }

        if (!(cloud_hull->points.empty()))
        {
            points.clear();

            unsigned int n_triangles = polygons.size();
            unsigned int n_vertices = cloud_hull->points.size();

            // Generating the polytope mesh
            poly_mesh.name = name_;

            for (int i = 0; i < n_vertices; i++)
            {
                geometry_msgs::Point pp;
                pp.x = cloud_hull->points[i].x;
                pp.y = cloud_hull->points[i].y;
                pp.z = cloud_hull->points[i].z;
                poly_mesh.mesh.vertices.push_back(pp);
            }

            for (int i = 0; i < n_triangles; i++){
                shape_msgs::MeshTriangle tri;
                tri.vertex_indices[0] = polygons[i].vertices[0];
                tri.vertex_indices[1] = polygons[i].vertices[1];
                tri.vertex_indices[2] = polygons[i].vertices[2];
                poly_mesh.mesh.triangles.push_back(tri);
            }

            // Also producing a vector of points for the visualization marker
            // Polygons are a vector of triangles represented by 3 indices
            // The indices correspond to points in cloud_hull
            // Therefore for each triangle in the polygon
            // we find its three vertices and extract their x y z coordinates
            for (int tri = 0; tri < n_triangles; ++tri)
            {
                pcl::Vertices triangle = polygons[tri];

                for (int var = 0; var < 3; ++var)
                {
                    geometry_msgs::Point pp;
                    pp.x = cloud_hull->points[triangle.vertices[var]].x;
                    pp.y = cloud_hull->points[triangle.vertices[var]].y;
                    pp.z = cloud_hull->points[triangle.vertices[var]].z;
                    points.push_back(pp);
                }
            }
        }
        else
        {
            ROS_WARN("plot: Hull empty");
            return false;
        }

        return true;
    }

    void Polytope::transformCartesian(Eigen::Matrix<double, 3, Eigen::Dynamic> Jp, Eigen::Vector3d P)
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

        // Changing state, so make sure to re-compute volume
        volume_ = computeVolume();
    }

    Polytope Polytope::slice(std::string name,
                             constrained_manipulability::SLICING_PLANE index,
                             double plane_width) const
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
        bHrep_other[(int)index] = plane_width;
        bHrep_other[((int)index) + 3] = plane_width;

        // Concacentate the polytope constraints to get the intersection
        return getPolytopeIntersection(name, AHrep_other, bHrep_other);
    }

    Polytope Polytope::getPolytopeIntersection(std::string name, const Eigen::MatrixXd &AHrep_other, const Eigen::VectorXd &bHrep_other) const
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
}