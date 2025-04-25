#include <gtest/gtest.h>

TEST(mil_tools_geometry_spline, spline_test)
{
    // Test case from: https://ece.uwaterloo.ca/~dwharder/C++/CQOST/Spline/
    // x, y, z, w
    Rotation r1{ 0.6, 0.8, 0, 0 };
    Rotation r2{ 0, 0.6, 0.8, 0 };
    Rotation r3{ 0.6, 0, 0.8, 0 };
    Rotation r4{ -0.8, 0, -0.6, 0 };

    std::array<Rotation, 4> rotations{ r1, r2, r3, r4 };
    Spline<4> spline(rotations);

    // 0.5
    std::array<Rotation, 31> expected_rotations{
        Rotation{ 0.6, 0.8, 0, 0 },
        Rotation{ 0.56678, 0.817286, 0.103943, 0 },
        Rotation{ 0.534208, 0.817053, 0.216904, 0 },
        Rotation{ 0.497805, 0.800752, 0.333147, 0 },
        Rotation{ 0.453954, 0.770941, 0.44674, 0 },
        Rotation{ 0.400276, 0.731562, 0.551901, 0 },
        Rotation{ 0.335789, 0.68791, 0.643448, 0 },
        Rotation{ 0.260903, 0.646293, 0.717102, 0 },
        Rotation{ 0.177418, 0.613502, 0.769505, 0 },
        Rotation{ 0.0886782, 0.596162, 0.797952, 0 },
        Rotation{ 0, 0.6, 0.8, 0 },
        Rotation{ 0.0789068, 0.491054, 0.867548, 0 },
        Rotation{ 0.152568, 0.410967, 0.898793, 0 },
        Rotation{ 0.222573, 0.357881, 0.906853, 0 },
        Rotation{ 0.29091, 0.325424, 0.899706, 0 },
        Rotation{ 0.358623, 0.304801, 0.882318, 0 },
        Rotation{ 0.425296, 0.285872, 0.85872, 0 },
        Rotation{ 0.488876, 0.257658, 0.833434, 0 },
        Rotation{ 0.545274, 0.208527, 0.811907, 0 },
        Rotation{ 0.586991, 0.126396, 0.799666, 0 },
        Rotation{ 0.6, 0, 0.8, 0 },
        Rotation{ 0.443058, 0.119923, 0.888436, 0 },
        Rotation{ 0.263784, 0.360515, 0.894677, 0 },
        Rotation{ -0.182842, 0.669303, 0.72014, 0 },
        Rotation{ -0.729334, 0.586345, 0.35252, 0 },
        Rotation{ -0.929021, 0.358023, 0.0934818, 0 },
        Rotation{ -0.977069, 0.206458, -0.0520745, 0 },
        Rotation{ -0.981986, 0.110608, -0.153196, 0 },
        Rotation{ -0.965122, 0.0489053, -0.257191, 0 },
        Rotation{ -0.916686, 0.012647, -0.399408, 0 },
        Rotation{ -0.8, 0, -0.6, 0 },
    };
    for (size_t i = 0; i < expected_rotations.size(); ++i)
    {
        EXPECT_NEAR(spline.get(i / 30.0).quat_x(), expected_rotations[i].quat_x(), 1e-5);
        EXPECT_NEAR(spline.get(i / 30.0).quat_y(), expected_rotations[i].quat_y(), 1e-5);
        EXPECT_NEAR(spline.get(i / 30.0).quat_z(), expected_rotations[i].quat_z(), 1e-5);
        EXPECT_NEAR(spline.get(i / 30.0).quat_w(), expected_rotations[i].quat_w(), 1e-5);
    }
}
