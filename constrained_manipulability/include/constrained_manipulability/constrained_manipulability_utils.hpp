#pragma once

#include <cassert>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "constrained_manipulability_interfaces/msg/matrix.hpp"

namespace constrained_manipulability
{
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> TransformVector;
typedef std::vector<Eigen::Matrix<double, 6, Eigen::Dynamic>> JacobianVector;

// Screw transform to move a twist from point a to point b, given vector L, a->b w.r.t. base frame
bool screwTransform(const Eigen::Matrix<double, 6, Eigen::Dynamic>& J0N_in,
                    const Eigen::Vector3d& L, 
                    Eigen::Matrix<double, 6, Eigen::Dynamic>& J0E_out);

// Convert a joint state message to a KDL joint array based on segment names
void jointStatetoKDLJointArray(const KDL::Chain& chain, 
                               const sensor_msgs::msg::JointState& joint_state, 
                               KDL::JntArray& kdl_joint_positions);

// Project translational Jacobian matrix along a vector
bool projectTranslationalJacobian(const Eigen::Vector3d& nT,
                                  const Eigen::Matrix<double, 6, Eigen::Dynamic>& J0N_in,
                                  Eigen::Matrix<double, 1, Eigen::Dynamic>& J0N_out);


void getCollisionShapes(const std::vector<int>& object_primitive, 
                        const std::vector<std::vector<double>>& obj_dimensions,
                        const std::vector<std::vector<double>>& obj_poses,
                        std::vector<shape_msgs::msg::SolidPrimitive>& shapes,
                        TransformVector& shape_poses);

template <typename T>
inline void printVector(const std::vector<T>& p, const std::string& name = "p= ")
{
    static rclcpp::Logger logger = rclcpp::get_logger("printVector");

    std::stringstream ss;
    ss << name << "[";
    for (size_t i = 0; i < p.size(); ++i)
    {
        ss << p[i];
        if (i != p.size() - 1)
        {
            ss << ", ";
        }
    }
    ss << "]";

    RCLCPP_INFO(logger, "%s", ss.str().c_str());
}

template <typename T>
inline void printVector(const std::vector<std::vector<T>>& p, const std::string& name = "p= ")
{
    static rclcpp::Logger logger = rclcpp::get_logger("printVector");

    std::stringstream ss;
    ss << name << "[";
    for (size_t i = 0; i < p.size(); ++i)
    {
        for (size_t j = 0; j < p[i].size(); ++j)
        {
            ss << p[i][j];
            if (j != p[i].size() - 1)
            {
                ss << ", ";
            }
        }
        if (i != p.size() - 1)
        {
            ss << ", ";
        }
    }
    ss << "]";

    RCLCPP_INFO(logger, "%s", ss.str().c_str());
}

template <typename T>
inline void vector2Matrix(const std::vector<T>& v1, std::vector<std::vector<T>>& M1,
                          unsigned int nrows, unsigned int ncols)
{
    assert(v1.size() == nrows * ncols);

    M1.resize(nrows);
    for (auto &i : M1)
    {
        i.resize(ncols);
    }
    for (unsigned int i = 0; i < nrows * ncols; i++)
    {
        int row = i / ncols;
        int col = i % ncols;
        M1[row][col] = v1[i];
    }
}

template <typename T>
inline void vector2Matrix(double x[], std::vector<std::vector<T>>& M1,
                          unsigned int nrows, unsigned int ncols)
{
    M1.resize(nrows);
    for (auto &i : M1)
    {
        i.resize(ncols);
    }
    for (unsigned int i = 0; i < nrows * ncols; i++)
    {
        int row = i / ncols;
        int col = i % ncols;
        M1[row][col] = x[i];
    }
}

template <typename T>
inline void matrix2Vector(const std::vector<std::vector<T>>& M1, std::vector<T>& v1,
                          unsigned int nrows, unsigned int ncols)
{
    assert(M1.size() == nrows);

    v1.resize(nrows * ncols);
    for (unsigned int i = 0; i < nrows * ncols; i++)
    {
        int row = i / ncols;
        int col = i % ncols;
        v1[i] = M1[row][col];
    }
}

template <typename T>
inline void printEigenTransform(const Eigen::Transform<T, 3, Eigen::Affine>& lTcom)
{
    static rclcpp::Logger logger = rclcpp::get_logger("printEigenTransform");

    RCLCPP_INFO(logger, " %f %f %f %f", lTcom.linear()(0, 0), lTcom.linear()(0, 1), lTcom.linear()(0, 2), lTcom.translation()(0));
    RCLCPP_INFO(logger, " %f %f %f %f", lTcom.linear()(1, 0), lTcom.linear()(1, 1), lTcom.linear()(1, 2), lTcom.translation()(1));
    RCLCPP_INFO(logger, " %f %f %f %f", lTcom.linear()(2, 0), lTcom.linear()(2, 1), lTcom.linear()(2, 2), lTcom.translation()(2));
    RCLCPP_INFO(logger, " %f %f %f %f", 0.0, 0.0, 0.0, 1.0);
}

template <typename T>
inline double errorNorm(const std::vector<T>& b, const std::vector<T>& a)
{
    assert(b.size() == a.size()); // ensure vectors are the same size

    double sum;
    int vec_size = b.size();
    for (int i = 0; i < vec_size; ++i)
    {
        sum = sum + pow(a[i] - b[i], 2);
    }
    return pow(sum, 0.5);
}

template <typename T>
inline double vectorNorm(const std::vector<T>& a)
{
    double sum(0.0);
    int vec_size = a.size();
    for (int i = 0; i < vec_size; ++i)
    {
        sum = sum + pow(a[i], 2);
    }
    return pow(sum, 0.5);
}

template <typename T>
inline void getParameter(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& val)
{
    if (node->has_parameter(param_name))
    {
        node->get_parameter(param_name, val);
        RCLCPP_INFO(node->get_logger(), "Parameter %s exists with value %s", param_name.c_str(), std::to_string(val).c_str());
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Parameter %s does not exist", param_name.c_str());
    }
}

template <typename T>
inline std::vector<T> parseVector(const std::string& str) {
    std::vector<T> result;
    std::stringstream ss(str);
    T value;
    while (ss >> value) {
        result.push_back(value);
        if (ss.peek() == ',') {
            ss.ignore();
        }
    }
    return result;
}

template <typename T>
inline std::vector<std::vector<T>> parseNestedVector(const std::vector<std::string>& strs) {
    std::vector<std::vector<T>> result;
    for (const auto &str : strs) {
        result.push_back(parseVector<T>(str));
    }
    return result;
}

inline std::vector<double> eigenAffineToVectorPosAngleAxis(const Eigen::Affine3d& T)
{
    Eigen::VectorXd pos = T.translation();
    Eigen::AngleAxisd angles_axis(T.linear());
    double a = angles_axis.angle();

    Eigen::Vector3d axis = angles_axis.axis();
    std::vector<double> p = {pos(0), pos(1), pos(2), a, axis(0), axis(1), axis(2)};
    return p;
}

inline std::vector<double> eigenAffineToVectorPosQuatwxyz(const Eigen::Affine3d& T)
{
    Eigen::VectorXd pos = T.translation();
    Eigen::Quaterniond quat(T.linear());
    std::vector<double> p = {pos(0), pos(1), pos(2), quat.w(), quat.x(), quat.y(), quat.z()};
    return p;
}

inline std::vector<double> eigenToVector(const Eigen::VectorXd& x)
{
    return std::vector<double>(x.data(), x.data() + x.size());
}

inline constrained_manipulability_interfaces::msg::Matrix eigenToMatrix(const Eigen::MatrixXd& A)
{
    constrained_manipulability_interfaces::msg::Matrix mat;
    mat.rows = A.rows();
    mat.columns = A.cols();
    mat.data.resize(A.size());

    int ii = 0;
    for (uint i = 0; i < A.rows(); ++i)
    {
        for (int j = 0; j < A.cols(); ++j)
        {
            mat.data[ii++] = A.coeff(i, j);
        }
    }
    return mat;
}

inline Eigen::VectorXd vectorToEigen(const std::vector<double>& x)
{
    return Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
}

inline Eigen::MatrixXd vectorToEigenMatrix(const std::vector<double>& x, int n)
{
    return Eigen::Map<const Eigen::MatrixXd>(x.data(), x.size(), x.size() / n);
}

inline Eigen::MatrixXd vectorToEigenMatrix(const Eigen::VectorXd& x, int n)
{
    return Eigen::Map<const Eigen::MatrixXd>(x.data(), n, x.size() / n);
}
} // namespace constrained_manipulability