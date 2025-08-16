#include "mil_tools/geometry/movement_utils.hpp"

#include <gtest/gtest.h>

#include "mil_tools/units/symbols.hpp"

using namespace mil::geometry;

TEST(mil_tools_movement_utils, composition)
{
    Movement m1 = forward(1 * m);
    EXPECT_EQ(m1.position().x(), 1);
}
