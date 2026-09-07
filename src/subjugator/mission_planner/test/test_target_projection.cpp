#include <gtest/gtest.h>

#include "target_projection.hpp"

using target_projection::CameraFrame;
using target_projection::goal_base_xy;
using target_projection::project_to_plane;
using target_projection::Vec2;
using target_projection::Vec3;

namespace
{
// Camera 0.5 m above the plane, looking straight down, image axes aligned with
// world x (right) and y (down). aspect = 600/960 = 0.625 (URDF down_cam).
CameraFrame straight_down()
{
    return CameraFrame{ /*origin*/ { 0.0, 0.0, 0.5 },
                        /*right*/ { 1.0, 0.0, 0.0 },
                        /*down*/ { 0.0, 1.0, 0.0 },
                        /*forward*/ { 0.0, 0.0, -1.0 } };
}
}  // namespace

TEST(TargetProjection, CenteredPixelHitsDirectlyBelow)
{
    auto hit = project_to_plane(0.0, 0.0, 1.5707963 /*90deg*/, 0.625, straight_down(), 0.0);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->x, 0.0, 1e-9);
    EXPECT_NEAR(hit->y, 0.0, 1e-9);
}

TEST(TargetProjection, EdgePixelProjectsByHeightAndFov)
{
    // hfov=90deg -> tan(45)=1; ex=0.5 -> tx=0.5; forward=(0,0,-1) so
    // dir=(0.5,0,-1), t=0.5, x = 0.5*0.5 = 0.25 m.
    auto hit = project_to_plane(0.5, 0.0, 1.5707963, 0.625, straight_down(), 0.0);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->x, 0.25, 1e-6);
    EXPECT_NEAR(hit->y, 0.0, 1e-6);
}

TEST(TargetProjection, VerticalEdgePixelScalesByAspect)
{
    // hfov=90deg -> tan(45)=1; ey=0.5, aspect=0.625 -> ty = 0.5*1*0.625 = 0.3125.
    // forward=(0,0,-1), down=(0,1,0) so dir=(0,0.3125,-1), t=0.5 (height 0.5m),
    // y = 0.5*0.3125 = 0.15625 m. Guards the aspect factor on the y axis.
    auto hit = project_to_plane(0.0, 0.5, 1.5707963, 0.625, straight_down(), 0.0);
    ASSERT_TRUE(hit.has_value());
    EXPECT_NEAR(hit->x, 0.0, 1e-6);
    EXPECT_NEAR(hit->y, 0.15625, 1e-6);
}

TEST(TargetProjection, RayParallelToPlaneReturnsNullopt)
{
    CameraFrame cam = straight_down();
    cam.forward = Vec3{ 1.0, 0.0, 0.0 };  // looking horizontally
    EXPECT_FALSE(project_to_plane(0.0, 0.0, 1.5707963, 0.625, cam, 0.0).has_value());
}

TEST(TargetProjection, PlaneBehindCameraReturnsNullopt)
{
    // Plane above a downward-looking camera -> t < 0.
    EXPECT_FALSE(project_to_plane(0.0, 0.0, 1.5707963, 0.625, straight_down(), 1.0).has_value());
}

TEST(TargetProjection, GripperOffsetShiftsGoalBase)
{
    // Gripper 0.1 m ahead (+x body); yaw 0 -> base is 0.1 behind the target.
    Vec2 base = goal_base_xy(Vec2{ 1.0, 1.0 }, 0.0, 0.1, 0.0);
    EXPECT_NEAR(base.x, 0.9, 1e-9);
    EXPECT_NEAR(base.y, 1.0, 1e-9);
}

TEST(TargetProjection, GripperOffsetRotatesWithYaw)
{
    // Same offset at yaw 90deg -> body +x points to world +y.
    Vec2 base = goal_base_xy(Vec2{ 1.0, 1.0 }, 1.5707963, 0.1, 0.0);
    EXPECT_NEAR(base.x, 1.0, 1e-6);
    EXPECT_NEAR(base.y, 0.9, 1e-6);
}
