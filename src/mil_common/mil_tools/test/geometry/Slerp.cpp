#include "mil_tools/geometry/Slerp.hpp"

#include <gtest/gtest.h>

#include <array>
#include <iostream>

using namespace mil::geometry;

TEST(mil_tools_rotation_slerp, computed_test)
{
    Rotation r1{ 0.6, 0.8, 0, 0 };
    Rotation r2{ -0.8, 0, -0.6, 0 };

    // no longer using these values since they are give a longer way to desired point
    // expected values from:
    // https://ece.uwaterloo.ca/~dwharder/C++/CQOST/Slerp/
    Slerp slerp(r1, r2);
    std::array<Rotation, 11> rotations{
        Rotation{ 0.6, 0.8, 0.0, 0.0 },
        Rotation{ 0.658905, 0.74867, 0.0730518, 0.0 },
        Rotation{ 0.710272, 0.688775, 0.145268, 0.0 },
        Rotation{ 0.753512, 0.621, 0.215822, 0.0 },
        Rotation{ 0.788131, 0.546119, 0.283906, 0.0 },
        Rotation{ 0.813733, 0.464991, 0.348743, 0.0 },
        Rotation{ 0.830026, 0.378542, 0.409589, 0.0 },
        Rotation{ 0.836821, 0.287762, 0.46575, 0.0 },
        Rotation{ 0.834043, 0.19369, 0.516581, 0.0 },
        Rotation{ 0.821722, 0.0974024, 0.561503, 0.0 },
        Rotation{ 0.8, 0.0, 0.6, 0.0 },
    };
    for (size_t i = 0; i < rotations.size(); i++)
    {
        Rotation result = slerp.at(i / 10.0);
        EXPECT_NEAR(result.quat_x(), rotations[i].quat_x(), 1e-6);
        EXPECT_NEAR(result.quat_y(), rotations[i].quat_y(), 1e-6);
        EXPECT_NEAR(result.quat_z(), rotations[i].quat_z(), 1e-6);
        EXPECT_NEAR(result.quat_w(), rotations[i].quat_w(), 1e-6);
    }
}
