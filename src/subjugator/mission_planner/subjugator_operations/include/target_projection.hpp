#pragma once

#include <cmath>
#include <optional>

// Pure, ROS-free ray-cast geometry for the down-cam world-frame target lock
// (LockTargetXY). Header-only and dependency-free so it is unit-tested without
// ROS/BT, following the detection_gate.hpp / select_target_logic.hpp pattern.
namespace target_projection
{

struct Vec2
{
    double x{ 0.0 };
    double y{ 0.0 };
};

struct Vec3
{
    double x{ 0.0 };
    double y{ 0.0 };
    double z{ 0.0 };
};

// Camera pose in the world frame: an origin plus three orthonormal axes.
//   right   — world direction of +image-x (columns increasing)
//   down    — world direction of +image-y (rows increasing)
//   forward — optical viewing axis (+z, into the scene)
// The node builds this from odometry + the static camera mount transform; the
// math here is agnostic to how it was built.
struct CameraFrame
{
    Vec3 origin;
    Vec3 right;
    Vec3 down;
    Vec3 forward;
};

// Cast the ray through a pixel given by its normalized image error (ex, ey in
// [-1, 1]; 0 = image center) and intersect the horizontal plane z = plane_z.
// hfov = horizontal field of view (rad); aspect_h_over_w = image height/width.
// Returns the world (x, y) of the intersection, or nullopt when the ray is
// parallel to the plane or points away from it.
inline std::optional<Vec2> project_to_plane(double ex, double ey, double hfov, double aspect_h_over_w,
                                            CameraFrame const& cam, double plane_z)
{
    double const th = std::tan(hfov / 2.0);
    double const tx = ex * th;
    double const ty = ey * th * aspect_h_over_w;

    // Ray direction in world = forward + tx*right + ty*down.
    Vec3 const dir{ cam.forward.x + tx * cam.right.x + ty * cam.down.x,
                    cam.forward.y + tx * cam.right.y + ty * cam.down.y,
                    cam.forward.z + tx * cam.right.z + ty * cam.down.z };

    if (std::abs(dir.z) < 1e-6)
    {
        return std::nullopt;  // ray parallel to the plane
    }
    double const t = (plane_z - cam.origin.z) / dir.z;
    if (t <= 0.0)
    {
        return std::nullopt;  // plane is behind the camera
    }
    return Vec2{ cam.origin.x + t * dir.x, cam.origin.y + t * dir.y };
}

// Where to command the sub's base (world XY) so a gripper at body-frame offset
// (gx, gy) ends up over target_world, given the sub's yaw (rad):
//   gripper_world = base + R(yaw)*(gx, gy)  ==>  base = target - R(yaw)*(gx, gy)
inline Vec2 goal_base_xy(Vec2 target_world, double sub_yaw, double gx, double gy)
{
    double const c = std::cos(sub_yaw);
    double const s = std::sin(sub_yaw);
    double const wx = c * gx - s * gy;
    double const wy = s * gx + c * gy;
    return Vec2{ target_world.x - wx, target_world.y - wy };
}

}  // namespace target_projection
