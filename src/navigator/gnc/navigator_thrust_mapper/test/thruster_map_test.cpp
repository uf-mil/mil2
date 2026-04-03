// Copyright 2025 University of Florida Machine Intelligence Laboratory
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "navigator_thrust_mapper/thruster_map.h"

#include <gtest/gtest.h>

#include <cmath>
#include <iomanip>
#include <iostream>

TEST(SimpleTest, ThrusterMap)
{
    std::vector<std::string> names = { "FL", "FR", "BL", "BR" };
    std::vector<std::array<double, 2>> positions = {
        { 0.5, 0.5 },   // Front-Left
        { 0.5, -0.5 },  // Front-Right
        { -0.5, 0.5 },  // Back-Left
        { -0.5, -0.5 }  // Back-Right
    };
    std::vector<double> angles = { 0.0, 0.0, 0.0, 0.0 };  // All aligned forward
    std::array<double, 2> force_limit = { 250.0, -250.0 };
    std::array<double, 2> center_of_mass = { 0.0, 0.0 };

    navigator_thrust_mapper::ThrusterMap thruster_map(names, positions, angles, force_limit, center_of_mass);

    std::array<double, 3> wrench = { 100.0, 0.0, 0.0 };
    std::vector<double> thrusts = thruster_map.wrench_to_thrusts(wrench);

    wrench = { 0.0, 50.0, 0.0 };  // Pure sway
    thrusts = thruster_map.wrench_to_thrusts(wrench);

    // Test with yaw
    wrench = { 0.0, 0.0, 25.0 };  // Pure yaw
    thrusts = thruster_map.wrench_to_thrusts(wrench);
}
