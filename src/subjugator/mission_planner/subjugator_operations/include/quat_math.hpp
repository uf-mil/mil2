#pragma once

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>

// Pure quaternion helpers shared by the bearing/pole-tracking BT nodes
// (HoneBearing, TrackBestPair, TrackLargestPoles). Every one of them was
// hand-rolling the same Hamilton product, yaw-delta construction, and
// normalization. Kept header-only and ROS-message-only (no BT/rclcpp) so it is
// unit-testable in isolation, matching the detection_gate.hpp / search_pattern.hpp
// convention.
namespace quat_math
{

#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

// Renormalize in place; a degenerate (near-zero-norm) quaternion collapses to
// identity so downstream orientation goals are always well-formed.
inline void normalize(geometry_msgs::msg::Quaternion& q)
{
    double const n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-12)
    {
        q.x = q.y = q.z = 0.0;
        q.w = 1.0;
        return;
    }
    q.x /= n;
    q.y /= n;
    q.z /= n;
    q.w /= n;
}

// Rotation of yaw_deg about +Z (body yaw), as a unit quaternion.
inline geometry_msgs::msg::Quaternion yaw_delta(double yaw_deg)
{
    double const r = (yaw_deg * M_PI / 180.0) * 0.5;
    geometry_msgs::msg::Quaternion q{};
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(r);
    q.w = std::cos(r);
    return q;
}

// Hamilton product a * b.
inline geometry_msgs::msg::Quaternion multiply(geometry_msgs::msg::Quaternion const& a,
                                               geometry_msgs::msg::Quaternion const& b)
{
    geometry_msgs::msg::Quaternion out;
    out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return out;
}

}  // namespace quat_math
