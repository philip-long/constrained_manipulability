#include <Eigen/StdVector>
#include <shape_msgs/SolidPrimitive.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/package.h>

typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> TransformVector;
typedef std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> JacobianVector;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

namespace utility_functions
{

    template <typename T>
    void printVector(std::vector<T> p, std::string name = "p= ")
    {
        std::cout << name << "[";
        for (int var = 0; var < p.size(); ++var)
        {
            std::cout << p[var] << "  ";
        }
        std::cout << "]" << std::endl;
    }

    template <typename T>
    void printVector(std::vector<std::vector<T>> p, std::string name = "p")
    {
        std::cout << name << "[";
        for (int var = 0; var < p.size(); ++var)
        {
            printVector(p[var], "");
        }
        std::cout << "]" << std::endl;
    }

    template <typename T>
    void vector2Matrix(const std::vector<T> &v1,
                       std::vector<std::vector<T>> &M1,
                       unsigned int nrows,
                       unsigned int ncols)
    {
        ROS_ASSERT(v1.size() == nrows * ncols);
        M1.resize(nrows);
        for (auto &i : M1)
        {
            i.resize(ncols);
        }
        for (int i = 0; i < nrows * ncols; i++)
        {
            int row = i / ncols;
            int col = i % ncols;
            M1[row][col] = v1[i];
        }
    }

    template <typename T>
    void vector2Matrix(double x[],
                       std::vector<std::vector<T>> &M1,
                       unsigned int nrows,
                       unsigned int ncols)
    {
        M1.resize(nrows);
        for (auto &i : M1)
        {
            i.resize(ncols);
        }
        for (int i = 0; i < nrows * ncols; i++)
        {
            int row = i / ncols;
            int col = i % ncols;
            M1[row][col] = x[i];
        }
    }

    template <typename T>
    void matrix2Vector(const std::vector<std::vector<T>> &M1, std::vector<T> &v1,
                       unsigned int nrows,
                       unsigned int ncols)
    {
        ROS_ASSERT(M1.size() == nrows);

        v1.resize(nrows * ncols);
        for (int i = 0; i < nrows * ncols; i++)
        {
            int row = i / ncols;
            int col = i % ncols;
            v1[i] = M1[row][col];
        }
    }

    template <typename T>
    void getParameter(std::string param_name, T &val)
    {
        if (ros::param::has(param_name))
        {
            ros::param::get(param_name, val);
            ROS_INFO_STREAM("Got Parameter, " << param_name << " value= " << val);
        }
        else
        {
            ROS_ERROR_STREAM(param_name << " parameter not found");
        }
    }

    template <typename T>
    void printEigenTransform(Eigen::Transform<T, 3, Eigen::Affine> lTcom)
    {
        std::cout << " " << lTcom.linear().row(0) << " " << lTcom.translation().row(0) << std::endl;
        std::cout << " " << lTcom.linear().row(1) << " " << lTcom.translation().row(1) << std::endl;
        std::cout << " " << lTcom.linear().row(2) << " " << lTcom.translation().row(2) << std::endl;
        std::cout << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << std::endl;
    }

    /// print elements std::vector of a type p
    /// elements of a must be compatible with std::cout<<
    template <typename T, typename U>
    void printTwoVectors(std::vector<T> p, std::vector<U> p2, std::string name = "p")
    {
        std::cout << name << std::endl;
        ROS_ASSERT(p.size() == p2.size());
        for (int var = 0; var < p.size(); ++var)
        {
            std::cout << "    " << p[var] << ": " << p2[var] << std::endl;
        }
    }

    template <typename T>
    double errorNorm(std::vector<T> b, std::vector<T> a)
    {
        assert(b.size() == a.size()); // ensure vectors are the same size

        double sum;
        for (int i = 0; i < b.size(); ++i)
        {
            sum = sum + pow(a[i] - b[i], 2);
        }
        return pow(sum, 0.5);
    }
    template <typename T>
    double vectorNorm(std::vector<T> a)
    {

        double sum(0.0);
        for (int i = 0; i < a.size(); ++i)
        {
            sum = sum + pow(a[i], 2);
        }
        return pow(sum, 0.5);
    }

    template <typename T>
    void getVectorParam(std::string parameter,
                        std::vector<T> &qparam)
    {
        XmlRpc::XmlRpcValue Axml;
        ros::param::get(parameter, Axml);

        ROS_ASSERT(Axml.getType() == Axml.TypeArray);
        qparam.resize(Axml.size());
        for (int i = 0; i < Axml.size(); ++i)
        {
            qparam[i] = Axml[i];
        }
    }

    template <typename T>
    void getVectorVectorParam(std::string parameter,
                              std::vector<std::vector<T>> &vectorofvectors)
    {
        XmlRpc::XmlRpcValue Axml;
        ros::param::get(parameter, Axml);

        ROS_ASSERT(Axml.getType() == Axml.TypeArray);
        vectorofvectors.resize(Axml.size());
        for (int i = 0; i < Axml.size(); ++i)
        {
            ROS_ASSERT(Axml[i].getType() == Axml.TypeArray);
            vectorofvectors[i].resize(Axml[i].size());
            for (int j = 0; j < Axml[i].size(); ++j)
            {
                vectorofvectors[i][j] = Axml[i][j];
            }
        }
    }
    static void getCollisionShapes(const std::vector<int> &object_primitive,
                                   const std::vector<std::vector<double>> &obj_dimensions,
                                   const std::vector<std::vector<double>> &obj_poses,
                                   std::vector<shape_msgs::SolidPrimitive> &shapes,
                                   TransformVector &shape_poses)
    {
        ROS_ASSERT(obj_dimensions.size() == object_primitive.size());
        ROS_ASSERT(obj_poses.size() == object_primitive.size());
        shapes.resize(object_primitive.size());
        shape_poses.resize(object_primitive.size());

        for (int i = 0; i < object_primitive.size(); ++i)
        {
            shapes[i].type = object_primitive[i];
            shapes[i].dimensions.resize(obj_dimensions[i].size());
            for (int j = 0; j < obj_dimensions[i].size(); ++j)
            {
                shapes[i].dimensions[j] = obj_dimensions[i][j];
            }
            Eigen::Vector3d p(obj_poses[i][0], obj_poses[i][1], obj_poses[i][2]);
            Eigen::Quaterniond q(obj_poses[i][3], obj_poses[i][4], obj_poses[i][5], obj_poses[i][6]);
            q.normalize();
            shape_poses[i].translation() = p;
            shape_poses[i].linear() = q.toRotationMatrix();
        }
    }

    static std::vector<double> eigenAffineToVectorPosAngleAxis(const Eigen::Affine3d &T)
    {
        Eigen::VectorXd pos = T.translation();
        Eigen::AngleAxisd angles_axis(T.linear());
        double a = angles_axis.angle();

        Eigen::Vector3d axis = angles_axis.axis();
        std::vector<double> p = {pos(0), pos(1), pos(2), a, axis(0), axis(1), axis(2)};
        return p;
    }

    static std::vector<double> eigenAffineToVectorPosQuatwxyz(const Eigen::Affine3d &T)
    {
        Eigen::VectorXd pos = T.translation();
        Eigen::Quaterniond quat(T.linear());
        std::vector<double> p = {pos(0), pos(1), pos(2), quat.w(), quat.x(), quat.y(), quat.z()};
        return p;
    }

    static std::vector<double> eigenToVector(const Eigen::VectorXd &x)
    {
        return std::vector<double>(x.data(), x.data() + x.size());
    }

    static Eigen::VectorXd vectorToEigen(const std::vector<double> &x)
    {
        return Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    }

    static Eigen::MatrixXd vectorToEigenMatrix(const std::vector<double> &x, const int &n)
    {
        return Eigen::Map<const Eigen::MatrixXd>(x.data(), x.size(), x.size() / n);
    }

    static Eigen::MatrixXd vectorToEigenMatrix(const Eigen::VectorXd &x, const int &n)
    {
        return Eigen::Map<const Eigen::MatrixXd>(x.data(), n, x.size() / n);
    }
}
