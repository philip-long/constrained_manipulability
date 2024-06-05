#include "constrained_manipulability/constrained_manipulability_utils.hpp"

namespace constrained_manipulability
{
namespace {
// Skew symmetric matrix performing the cross product
inline bool skew(const Eigen::Vector3d& L, Eigen::Matrix<double, 3, 3>& skewL)
{
    skewL(0, 1) = -L(2);
    skewL(0, 2) = L(1);
    skewL(1, 2) = -L(0);
    skewL(1, 0) = -skewL(0, 1);
    skewL(2, 0) = -skewL(0, 2);
    skewL(2, 1) = -skewL(1, 2);
    return true;
}
} // namespace

// Screw transform to move a twist from point a to point b, given vector L, a->b w.r.t. base frame
bool screwTransform(const Eigen::Matrix<double, 6, Eigen::Dynamic>& J0N_in,
                    const Eigen::Vector3d& L, 
                    Eigen::Matrix<double, 6, Eigen::Dynamic>& J0E_out)
{
    J0E_out.setZero();
    Eigen::Matrix<double, 3, 3> Lhat;
    Lhat.setZero();
    skew(L, Lhat);
    Eigen::Matrix<double, 6, 6> screwL;
    screwL.setZero();
    screwL.topLeftCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    screwL.bottomRightCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    screwL.bottomLeftCorner(3, 3) = Eigen::Matrix<double, 3, 3>::Zero();
    screwL.topRightCorner(3, 3) = -Lhat;
    J0E_out = screwL * J0N_in;
    return true;
}

void getCollisionShapes(const std::vector<int>& object_primitive, 
                        const std::vector<std::vector<double>>& obj_dimensions,
                        const std::vector<std::vector<double>>& obj_poses,
                        std::vector<shape_msgs::msg::SolidPrimitive>& shapes,
                        TransformVector& shape_poses)
{
    unsigned int obj_primitive_size = object_primitive.size();
    assert(obj_dimensions.size() == obj_primitive_size);
    assert(obj_poses.size() == obj_primitive_size);

    shapes.resize(obj_primitive_size);
    shape_poses.resize(obj_primitive_size);

    for (unsigned int i = 0; i < obj_primitive_size; ++i)
    {
        shapes[i].type = object_primitive[i];
        shapes[i].dimensions.resize(obj_dimensions[i].size());
        for (unsigned int j = 0; j < obj_dimensions[i].size(); ++j)
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

// Convert a joint state message to a KDL joint array based on segment names
void jointStatetoKDLJointArray(const KDL::Chain& chain,
                               const sensor_msgs::msg::JointState& joint_states, 
                               KDL::JntArray& kdl_joint_positions)
{
    unsigned int jnt(0);
    unsigned int num_segments(chain.getNrOfSegments());
    for (unsigned int i = 0; i < num_segments; ++i)
    {
        KDL::Segment seg = chain.getSegment(i);
        KDL::Joint kdl_joint = seg.getJoint();
        for (unsigned int j = 0; j < joint_states.name.size(); ++j)
        {
            if (kdl_joint.getName() == joint_states.name[j])
            {
                kdl_joint_positions(jnt) = joint_states.position[j];
                jnt++;
            }
        }
    }
}

// Project translational Jacobian matrix along a vector
bool projectTranslationalJacobian(const Eigen::Vector3d& nT,
                                  const Eigen::Matrix<double, 6, Eigen::Dynamic>& J0N_in,
                                  Eigen::Matrix<double, 1, Eigen::Dynamic>& J0N_out)
{
    int n = J0N_in.cols();
    assert(J0N_in.rows() > 3);
    J0N_out.setZero();
    J0N_out = nT.transpose() * J0N_in.topLeftCorner(3, n);
    return true;
}

std::vector<double> eigenAffineToVectorPosAngleAxis(const Eigen::Affine3d& T)
{
    Eigen::VectorXd pos = T.translation();
    Eigen::AngleAxisd angles_axis(T.linear());
    double a = angles_axis.angle();

    Eigen::Vector3d axis = angles_axis.axis();
    std::vector<double> p = {pos(0), pos(1), pos(2), a, axis(0), axis(1), axis(2)};
    return p;
}

std::vector<double> eigenAffineToVectorPosQuatwxyz(const Eigen::Affine3d& T)
{
    Eigen::VectorXd pos = T.translation();
    Eigen::Quaterniond quat(T.linear());
    std::vector<double> p = {pos(0), pos(1), pos(2), quat.w(), quat.x(), quat.y(), quat.z()};
    return p;
}

constrained_manipulability_interfaces::msg::Matrix eigenToMatrix(const Eigen::MatrixXd& A)
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
} // namespace constrained_manipulability