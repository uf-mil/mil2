#include "mil_tools/geometry/Rotation.hpp"

#include <gtest/gtest.h>

using mil_tools::geometry::Axis;

TEST(mil_tools_geometry_rotation, identity)
{
    mil_tools::geometry::Rotation r;
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
    mil_tools::geometry::Rotation r1(q);
    mil_tools::geometry::Rotation r2(r1.quat());
    EXPECT_TRUE(r1 == r2);
    double r, p, y;
    tf2::Matrix3x3(tf2_q).getRPY(r, p, y);
    auto const &[rot_roll, rot_pitch, rot_yaw] = r1.rot_vec_tuple(Axis::X, Axis::Y, Axis::Z);
    // values from: https://quaternions.online/
    EXPECT_NEAR(0.527, rot_roll, 1e-2);
    EXPECT_NEAR(1.222, rot_pitch, 1e-2);
    EXPECT_NEAR(1.567, rot_yaw, 1e-2);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
