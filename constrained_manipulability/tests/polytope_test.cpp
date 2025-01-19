#include <gtest/gtest.h>

#include <Eigen/Eigen>

#include "constrained_manipulability/polytope.hpp"

TEST(PolytopeTest, DefaultConstructor) {
    constrained_manipulability::Polytope poly;
    EXPECT_FALSE(poly.isValid());
    EXPECT_EQ(poly.getName(), "invalid_polytope");
    EXPECT_EQ(poly.getVolume(), 0.0);
    EXPECT_TRUE(poly.getPoints().empty());
}

TEST(PolytopeTest, VertexSetConstructor) {
    Eigen::MatrixXd vertices(4, 3);
    vertices << 0, 0, 0, 
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
    Eigen::Vector3d offset(1, 1, 1);
    constrained_manipulability::Polytope poly("test_polytope", vertices, offset);

    EXPECT_TRUE(poly.isValid());
    EXPECT_EQ(poly.getName(), "test_polytope");
    // Volume should be computed
    EXPECT_GT(poly.getVolume(), 0.0);
    EXPECT_FALSE(poly.getPoints().empty());
}

TEST(PolytopeTest, AHrepConstructor) {
    Eigen::MatrixXd A(6, 3);
    A << 1, 0, 0,
         -1, 0, 0,
         0, 1, 0,
         0, -1, 0,
         4, 0, 0,
         0, 2, 0;
    Eigen::VectorXd b(6);
    b << 1, 1, 1, 1, 2, 2;
    Eigen::Vector3d offset(0, 0, 0);
    constrained_manipulability::Polytope poly("test_polytope", A, b, offset);

    EXPECT_TRUE(poly.isValid());
    EXPECT_EQ(poly.getName(), "test_polytope");
    // Volume should be computed
    EXPECT_GT(poly.getVolume(), 0.0);
    EXPECT_FALSE(poly.getPoints().empty());
}

TEST(PolytopeTest, TransformCartesian) {
    Eigen::MatrixXd vertices(4, 3);
    vertices << 0, 0, 0, 
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
    Eigen::Vector3d offset(1, 1, 1);
    constrained_manipulability::Polytope poly("test_transform", vertices, offset);

    Eigen::Matrix<double, 3, 3> Jp;
    Jp << 1, 0, 0,
          0, 1, 0,
          0, 0, 1;

    poly.transformCartesian(Jp);

    EXPECT_TRUE(poly.isValid());
    // Volume should remain positive after transformation
    EXPECT_GT(poly.getVolume(), 0.0);
}

TEST(PolytopeTest, Slice) {
    Eigen::MatrixXd vertices(8, 3);
    vertices << 0, 1, 1, 
                0, 0, 1, 
                1, 0, 0, 
                0, 1, 1, 
                0, 0, 1, 
                1, 0, 0, 
                0, 0, 1, 
                1, 1, 1;
    Eigen::Vector3d offset(0, 0, 0);
    constrained_manipulability::Polytope poly("test_slice", vertices, offset);

    double plane_width = 0.1;
    constrained_manipulability::Polytope sliced = poly.slice("sliced_polytope", 
                                                             constrained_manipulability::SLICING_PLANE::XY_PLANE, 
                                                             plane_width);

    EXPECT_TRUE(sliced.isValid());
    EXPECT_EQ(sliced.getName(), "sliced_polytope");
    // Sliced polytope should have a positive volume
    EXPECT_GT(sliced.getVolume(), 0.0);
}

TEST(PolytopeTest, PolytopeIntersection) {
    Eigen::MatrixXd A1(4, 3);
    A1 << 1, 0, 0,
          -1, 0, 0,
          0, 1, 0,
          0, -1, 0;
    Eigen::VectorXd b1(4);
    b1 << 1, 1, 1, 1;

    Eigen::MatrixXd A2(4, 3);
    A2 << 1, 0, 0,
          -1, 0, 0,
          0, 1, 0,
          0, -1, 0;
    Eigen::VectorXd b2(4);
    b2 << 0.5, 0.5, 0.5, 0.5;

    Eigen::Vector3d offset(0, 0, 0);
    constrained_manipulability::Polytope poly1("poly1", A1, b1, offset);
    constrained_manipulability::Polytope intersection = poly1.getPolytopeIntersection("intersection", A2, b2);

    EXPECT_TRUE(intersection.isValid());
    EXPECT_EQ(intersection.getName(), "intersection");
    // Intersection should have a positive volume
    EXPECT_GT(intersection.getVolume(), 0.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}