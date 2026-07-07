#include <gtest/gtest.h>

#include <cmath>

#include "quat_math.hpp"

namespace
{
constexpr double kEps = 1e-9;

double norm(geometry_msgs::msg::Quaternion const& q)
{
    return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}
}  // namespace

TEST(QuatMath, YawDeltaIsUnitAndAboutZ)
{
    auto q = quat_math::yaw_delta(90.0);
    EXPECT_NEAR(q.x, 0.0, kEps);
    EXPECT_NEAR(q.y, 0.0, kEps);
    EXPECT_NEAR(q.z, std::sin(M_PI / 4.0), kEps);  // sin(45 deg)
    EXPECT_NEAR(q.w, std::cos(M_PI / 4.0), kEps);
    EXPECT_NEAR(norm(q), 1.0, kEps);
}

TEST(QuatMath, MultiplyByIdentityIsUnchanged)
{
    geometry_msgs::msg::Quaternion id{};
    id.w = 1.0;
    auto a = quat_math::yaw_delta(37.0);
    auto out = quat_math::multiply(a, id);
    EXPECT_NEAR(out.x, a.x, kEps);
    EXPECT_NEAR(out.y, a.y, kEps);
    EXPECT_NEAR(out.z, a.z, kEps);
    EXPECT_NEAR(out.w, a.w, kEps);
}

TEST(QuatMath, YawDeltasCompose)
{
    // yaw_delta(30) * yaw_delta(60) == yaw_delta(90) for rotations about the same axis
    auto composed = quat_math::multiply(quat_math::yaw_delta(30.0), quat_math::yaw_delta(60.0));
    quat_math::normalize(composed);
    auto direct = quat_math::yaw_delta(90.0);
    EXPECT_NEAR(composed.z, direct.z, kEps);
    EXPECT_NEAR(composed.w, direct.w, kEps);
}

TEST(QuatMath, NormalizeScalesToUnit)
{
    geometry_msgs::msg::Quaternion q{};
    q.x = 0.0;
    q.y = 0.0;
    q.z = 3.0;
    q.w = 4.0;  // norm 5
    quat_math::normalize(q);
    EXPECT_NEAR(norm(q), 1.0, kEps);
    EXPECT_NEAR(q.z, 0.6, kEps);
    EXPECT_NEAR(q.w, 0.8, kEps);
}

TEST(QuatMath, NormalizeDegenerateCollapsesToIdentity)
{
    geometry_msgs::msg::Quaternion q{};  // all zero -> norm 0
    quat_math::normalize(q);
    EXPECT_NEAR(q.x, 0.0, kEps);
    EXPECT_NEAR(q.y, 0.0, kEps);
    EXPECT_NEAR(q.z, 0.0, kEps);
    EXPECT_NEAR(q.w, 1.0, kEps);
}
