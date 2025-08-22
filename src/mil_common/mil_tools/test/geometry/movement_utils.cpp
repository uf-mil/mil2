#include "mil_tools/geometry/movement_utils.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "mil_tools/units/symbols.hpp"

using namespace mil::geometry;

TEST(mil_tools_movement_utils, individual)
{
    EXPECT_EQ(forward(1 * m), Movement{ .position = Eigen::Vector3d(1, 0, 0) });
    EXPECT_EQ(backward(1 * m), Movement{ .position = Eigen::Vector3d(-1, 0, 0) });
}

TEST(mil_tools_movement_utils, precision)
{
    EXPECT_EQ(forward(1 * m).forward(1.0f * m).forward(1.0d * m), Movement{ .position = Eigen::Vector3d(3, 0, 0) });
}

TEST(mil_tools_movement_utils, composition)
{
    EXPECT_EQ(forward(2 * m).left(1 * m).down(3 * m), Movement{ .position = Eigen::Vector3d(2, -1, -3) });
}

TEST(mil_tools_movement_utils, units)
{
    // Not using doubles here will become a problem (since you cannot cleanly convert 1 ft to <int> m)
    Movement movement = forward(1.0 * m).left(1.0 * ft);
    EXPECT_EQ(movement.position.x(), 1);
    EXPECT_NEAR(movement.position.y(), -0.3048, 1e-4);
}
