#include "mil_tools/geometry/Rotation.hpp"

#include <gtest/gtest.h>

#include <iostream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using mil_tools::geometry::Axis;
using mil_tools::geometry::Rotation;

TEST(mil_tools_geometry_rotation, identity)
{
    Rotation r;
    EXPECT_EQ(r.quat_w(), 1);
    EXPECT_EQ(r.quat_x(), 0);
    EXPECT_EQ(r.quat_y(), 0);
    EXPECT_EQ(r.quat_z(), 0);
    EXPECT_EQ(r.roll(), 0);
    EXPECT_EQ(r.pitch(), 0);
    EXPECT_EQ(r.yaw(), 0);
    Eigen::Vector3d zero_vec{ 0, 0, 0 };
    EXPECT_EQ(zero_vec, r.rot_vec());
}

TEST(mil_tools_geometry_rotation, conversion)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.542;
    q.y = 0.242;
    q.z = 0.664;
    q.w = 0.455;
    tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
    tf2_q.normalize();
    Rotation r1(q);
    Rotation r2(r1.quat());
    EXPECT_TRUE(r1 == r2);
    double r, p, y;
    tf2::Matrix3x3(tf2_q).getRPY(r, p, y);
    auto const &[rot_roll, rot_pitch, rot_yaw] = r1.rot_vec_tuple(Axis::X, Axis::Y, Axis::Z);
    // values from: https://quaternions.online/
    EXPECT_NEAR(0.527, rot_roll, 1e-2);
    EXPECT_NEAR(1.222, rot_pitch, 1e-2);
    EXPECT_NEAR(1.567, rot_yaw, 1e-2);
}

TEST(mil_tools_geometry_rotation, msg)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.542;
    q.y = 0.242;
    q.z = 0.664;
    q.w = 0.455;
    Rotation r(q);
    auto q_msg = r.quat_msg();
    EXPECT_EQ(q.x, q_msg.x);
    EXPECT_EQ(q.y, q_msg.y);
    EXPECT_EQ(q.z, q_msg.z);
    EXPECT_EQ(q.w, q_msg.w);
    auto rot_vec_msg = r.rot_vec_msg();
    auto rot_vec = r.rot_vec();
    EXPECT_EQ(rot_vec[0], rot_vec_msg.x);
    EXPECT_EQ(rot_vec[1], rot_vec_msg.y);
    EXPECT_EQ(rot_vec[2], rot_vec_msg.z);
}

TEST(mil_tools_geometry_rotation, operators)
{
    Rotation identity;
    EXPECT_TRUE(identity == Rotation());
    EXPECT_FALSE(identity != Rotation());
    Rotation composed = identity * identity;
    EXPECT_TRUE(composed == identity);

    // unary operator-() should return the inverse rotation
    Rotation r1{ Eigen::Vector3d{ 0.1, -0.2, 0.3 } };
    Rotation r1_inv{ -r1 };

    // r1 * -r1 should be identity
    Rotation should_be_identity = r1 * r1_inv;
    EXPECT_TRUE(should_be_identity.quat().isApprox(identity.quat(), 1e-9));

    // composition of two known rotations
    Rotation r2{ Eigen::Vector3d{ 0.0, 0.0, M_PI / 2 } };  // 90 deg yaw
    Rotation r3{ Eigen::Vector3d{ 0.0, M_PI / 2, 0.0 } };  // 90 deg pitch
    Rotation r4 = r2 * r3;
    Eigen::Quaterniond expected = r2.quat() * r3.quat();
    EXPECT_TRUE(r4.quat().isApprox(expected, 1e-2));

    // brief inequality test
    EXPECT_TRUE(r2 != r3);
    EXPECT_FALSE(r2 == r3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
