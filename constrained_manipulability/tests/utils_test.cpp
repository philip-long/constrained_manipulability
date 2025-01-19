#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "constrained_manipulability/constrained_manipulability_utils.hpp"

TEST(ConstrainedManipulabilityUtils, ScrewTransformTest)
{
    // Input vector for skew
    Eigen::Vector3d L(1.0, 2.0, 3.0);

    // Input Jacobian matrix
    Eigen::Matrix<double, 6, 4> J0N_in;
    J0N_in << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1,
              1, 0, 0, 0,
              0, 1, 0, 0;

    // Expected screw-transformed Jacobian
    Eigen::Matrix<double, 6, 4> expectedJ0E_out;
    expectedJ0E_out << 4, -2, 0, 0,
                       0, 2, 0, -3,
                       -1, 0, 1, 2,
                       0, 0, 0, 1,
                       1, 0, 0, 0,
                       0, 1, 0, 0;

    // Compute screw transformation
    Eigen::Matrix<double, 6, Eigen::Dynamic> actualJ0E_out;
    actualJ0E_out.resize(6, 4);

    ASSERT_TRUE(constrained_manipulability::screwTransform(J0N_in, L, actualJ0E_out));

    // Check correctness
    EXPECT_TRUE(actualJ0E_out.isApprox(expectedJ0E_out));
}

TEST(ConstrainedManipulabilityUtils, GetCollisionShapesTest)
{
    std::vector<int> object_primitive = {1, 2};
    std::vector<std::vector<double>> obj_dimensions = {{1.0}, {2.0, 3.0}};
    std::vector<std::vector<double>> obj_poses = {
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0}};

    std::vector<shape_msgs::msg::SolidPrimitive> shapes;
    constrained_manipulability::TransformVector shape_poses;

    constrained_manipulability::getCollisionShapes(object_primitive, 
                                                   obj_dimensions, 
                                                   obj_poses, 
                                                   shapes, 
                                                   shape_poses);

    ASSERT_EQ(shapes.size(), 2);
    ASSERT_EQ(shape_poses.size(), 2);

    EXPECT_EQ(shapes[0].type, 1);
    EXPECT_EQ(shapes[1].type, 2);

    EXPECT_EQ(shapes[0].dimensions.size(), 1);
    EXPECT_EQ(shapes[1].dimensions.size(), 2);

    EXPECT_DOUBLE_EQ(shapes[0].dimensions[0], 1.0);
    EXPECT_DOUBLE_EQ(shapes[1].dimensions[0], 2.0);
    EXPECT_DOUBLE_EQ(shapes[1].dimensions[1], 3.0);
}

TEST(ConstrainedManipulabilityUtils, JointStateToKDLJointArrayTest)
{
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(KDL::Joint("joint1", KDL::Joint::RotZ)));
    chain.addSegment(KDL::Segment(KDL::Joint("joint2", KDL::Joint::RotZ)));

    sensor_msgs::msg::JointState joint_state;
    joint_state.name = {"joint1", "joint2"};
    joint_state.position = {1.0, 2.0};

    KDL::JntArray kdl_joint_positions(2);
    constrained_manipulability::jointStatetoKDLJointArray(chain, joint_state, kdl_joint_positions);

    EXPECT_DOUBLE_EQ(kdl_joint_positions(0), 1.0);
    EXPECT_DOUBLE_EQ(kdl_joint_positions(1), 2.0);
}

TEST(ConstrainedManipulabilityUtils, ProjectTranslationalJacobianTest)
{
    Eigen::Vector3d nT(1.0, 0.0, 0.0);
    // INput Jacobian matrix
    Eigen::Matrix<double, 6, 4> J0N_in;
    J0N_in << 1, 2, 3, 4,
              5, 6, 7, 8,
              9, 10, 11, 12,
              0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;

    // Expected projection
    Eigen::Matrix<double, 1, 4> expectedJ0E_out;
    expectedJ0E_out << 1, 2, 3, 4;

    // Projection result
    Eigen::Matrix<double, 1, Eigen::Dynamic> actualJ0E_out;
    actualJ0E_out.resize(1, 4);

    ASSERT_TRUE(constrained_manipulability::projectTranslationalJacobian(nT, J0N_in, actualJ0E_out));

    EXPECT_TRUE(actualJ0E_out.isApprox(expectedJ0E_out));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}