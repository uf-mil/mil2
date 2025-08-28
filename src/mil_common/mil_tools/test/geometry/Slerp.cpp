#include "mil_tools/geometry/Slerp.hpp"

#include <gtest/gtest.h>

#include <array>
#include <iostream>

using namespace mil::geometry;

TEST(mil_tools_rotation_slerp, computed_test)
{
    Rotation r1{ 0.6, 0.8, 0, 0 };
    Rotation r2{ -0.8, 0, -0.6, 0 };

    // expected values from:
    // https://ece.uwaterloo.ca/~dwharder/C++/CQOST/Slerp/
    Slerp slerp(r1, r2);
    std::array<Rotation, 11> rotations{
        Rotation{ 0.6, 0.8, 0.0, 0.0 },
        Rotation{ 0.46714, 0.872923, -0.140664, 0.0 },
        Rotation{ 0.314307, 0.908523, -0.275314, 0.0 },
        Rotation{ 0.148035, 0.905278, -0.398192, 0.0 },
        Rotation{ -0.0245656, 0.863327, -0.504046, 0.0 },
        Rotation{ -0.196116, 0.784465, -0.588348, 0.0 },
        Rotation{ -0.359282, 0.672061, -0.647496, 0.0 },
        Rotation{ -0.507086, 0.530923, -0.678959, 0.0 },
        Rotation{ -0.633209, 0.367085, -0.681392, 0.0 },
        Rotation{ -0.732259, 0.187552, -0.654692, 0.0 },
        Rotation{ -0.8, 5.55112e-17, -0.6, 0.0 },
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
