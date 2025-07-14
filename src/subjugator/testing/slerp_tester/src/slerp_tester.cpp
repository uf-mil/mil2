#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "mil_tools/geometry/Rotation.hpp"
#include "mil_tools/geometry/Slerp.hpp"

// Utility to compute quaternion magnitude
double quaternionMagnitude(std::array<double, 4> const& q)
{
    return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

// Utility to print a quaternion with its magnitude
void printQuaternion(std::array<double, 4> const& q, std::string const& label)
{
    std::cout << label << ": [" << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3]
              << "], magnitude: " << quaternionMagnitude(q) << std::endl;
}

// Utility to check if values are close to zero (for handling floating point errors)
bool isNearZero(double val, double epsilon = 1e-15)
{
    return std::fabs(val) < epsilon;
}

// Test SLERP with various inputs
void testSlerp()
{
    std::cout << "\n=== TESTING SLERP NORMALIZATION ===" << std::endl;

    // Test 1: Basic SLERP between identity and 180° rotation around Z
    std::array<double, 4> identity = { 1, 0, 0, 0 };  // w, x, y, z
    std::array<double, 4> rotZ180 = { 0, 0, 0, 1 };

    auto slerp1 = mil_tools::geometry::Slerp(identity, rotZ180);
    std::cout << "Test 1: Identity to 180° Z rotation:" << std::endl;

    std::vector<double> checkpoints = { 0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0 };
    for (double t : checkpoints)
    {
        auto result = slerp1.at(t);
        std::cout << "t=" << t << ": ";
        std::cout << result << std::endl;
    }

    // Test 2: Very close quaternions
    std::cout << "\n=== TESTING NEARLY IDENTICAL QUATERNIONS ===" << std::endl;
    std::array<double, 4> q1 = { 0.9999, 0.01, 0.01, 0.01 };
    std::array<double, 4> q2 = { 0.9998, 0.01, 0.015, 0.01 };

    // Normalize inputs (to ensure valid quaternions)
    double mag1 = quaternionMagnitude(q1);
    double mag2 = quaternionMagnitude(q2);
    for (int i = 0; i < 4; i++)
    {
        q1[i] /= mag1;
        q2[i] /= mag2;
    }

    printQuaternion(q1, "Starting quaternion");
    printQuaternion(q2, "Ending quaternion");

    auto slerp2 = mil_tools::geometry::Slerp(q1, q2);
    std::cout << "Interpolation between very close quaternions:" << std::endl;
    for (double t : checkpoints)
    {
        auto result = slerp2.at(t);
        std::cout << "t=" << t << ": ";
        std::cout << result << std::endl;
    }

    // Test 3: Check for floating point errors near zero
    std::cout << "\n=== TESTING FLOATING POINT PRECISION ===" << std::endl;
    std::array<double, 4> q3 = { 1, 0, 0, 0 };
    std::array<double, 4> q4 = { 0, 1, 0, 0 };

    auto slerp3 = mil_tools::geometry::Slerp(q3, q4);
    auto result = slerp3.at(0.5);
    auto r_quat = result.quat();

    std::cout << "Testing for small values that should be zero:" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        if (isNearZero(r_quat[i]))
        {
            std::cout << "Component " << i << " is very small: " << r_quat[i] << " (might be intended as 0)"
                      << std::endl;
        }
    }
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    testSlerp();

    return 0;
}
