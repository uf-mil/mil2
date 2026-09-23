// Unit tests for the hard/soft-iron correction math.
//
// These exercise compensate() in compensator.hpp directly (the same code the
// live node calls), so no ROS node or spinning is required. Parameter
// validation (size + invertibility checks) stays inline in the node
// constructor and is not directly covered here.

#include <gtest/gtest.h>

#include <magnetic_compensation/compensator.hpp>

using mil::magnetic_compensation::compensate;

TEST(Compensate, IdentityLeavesReadingUnchanged)
{
    Eigen::Matrix3d scale_inv = Eigen::Matrix3d::Identity();
    Eigen::Vector3d shift = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw(1.0, 2.0, 3.0);
    EXPECT_TRUE(compensate(scale_inv, shift, raw).isApprox(raw));
}

TEST(Compensate, PureShiftIsSubtracted)
{
    Eigen::Matrix3d scale_inv = Eigen::Matrix3d::Identity();
    Eigen::Vector3d shift(0.5, -1.0, 2.0);
    Eigen::Vector3d raw(1.0, 2.0, 3.0);
    Eigen::Vector3d expected(0.5, 3.0, 1.0);
    EXPECT_TRUE(compensate(scale_inv, shift, raw).isApprox(expected));
}

TEST(Compensate, DiagonalScaleIsInverted)
{
    // scale = diag(2,2,2) -> scale_inverse = diag(0.5,0.5,0.5)
    Eigen::Matrix3d scale_inv;
    scale_inv << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5;
    Eigen::Vector3d shift = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw(2.0, 4.0, 6.0);
    Eigen::Vector3d expected(1.0, 2.0, 3.0);
    EXPECT_TRUE(compensate(scale_inv, shift, raw).isApprox(expected));
}
