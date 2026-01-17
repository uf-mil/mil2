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

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace navigator_thrust_mapper;

int main()
{
    std::cout << "=== ThrusterMap C++ Conversion Test ===" << std::endl << std::endl;

    // Test 1: VRX Force to Command Conversion
    std::cout << "Test 1: VRX Force-to-Command Conversion" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    double test_forces[] = { 0.0, 1.0, 3.27398, 50.0, 100.0, 250.0, 300.0, -50.0, -100.0, -150.0 };

    for (double force : test_forces)
    {
        double cmd = vrx_force_to_command_scalar(force);
        std::cout << "  Force: " << std::setw(8) << std::fixed << std::setprecision(2) << force
                  << " N  →  Command: " << std::setw(8) << std::fixed << std::setprecision(4) << cmd << std::endl;
    }
    std::cout << std::endl;

    // Test 2: Vectorized force conversion
    std::cout << "Test 2: Vectorized Force Conversion" << std::endl;
    std::cout << "-----------------------------------" << std::endl;

    Eigen::VectorXd forces_vec(4);
    forces_vec << 0.0, 50.0, 100.0, 250.0;

    Eigen::VectorXd cmds = vrx_force_to_command(forces_vec);
    std::cout << "  Input forces (N):  " << forces_vec.transpose() << std::endl;
    std::cout << "  Output commands:   " << cmds.transpose() << std::endl;
    std::cout << std::endl;

    // Test 3: Linear force-to-command generator
    std::cout << "Test 3: Linear Force-to-Command Generator" << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    double ratio = 0.1;
    auto linear_converter = generate_linear_force_to_command(ratio);

    Eigen::VectorXd test_forces_linear(4);
    test_forces_linear << 10.0, 20.0, 50.0, 100.0;

    Eigen::VectorXd linear_result = linear_converter(test_forces_linear);
    std::cout << "  Ratio: " << ratio << std::endl;
    std::cout << "  Input forces (N):  " << test_forces_linear.transpose() << std::endl;
    std::cout << "  Output commands:   " << linear_result.transpose() << std::endl;
    std::cout << std::endl;

    // Test 4: ThrusterMap initialization and wrench-to-thrusts
    std::cout << "Test 4: ThrusterMap - Wrench to Thrusts Conversion" << std::endl;
    std::cout << "---------------------------------------------------" << std::endl;

    std::vector<std::string> names = { "FL", "FR", "BL", "BR" };
    std::vector<std::array<double, 2>> positions = {
        { 0.5, 0.5 },   // Front-Left
        { 0.5, -0.5 },  // Front-Right
        { -0.5, 0.5 },  // Back-Left
        { -0.5, -0.5 }  // Back-Right
    };
    std::vector<double> angles = { 0.0, 0.0, 0.0, 0.0 };  // All aligned forward
    std::vector<std::string> joints = { "fl_joint", "fr_joint", "bl_joint", "br_joint" };

    auto force_to_cmd = generate_linear_force_to_command(1.0);
    std::array<double, 2> force_limit = { 250.0, -250.0 };
    std::array<double, 2> com = { 0.0, 0.0 };

    try
    {
        ThrusterMap thruster_map(names, positions, angles, force_to_cmd, force_limit, com, joints);

        std::cout << "  ThrusterMap created successfully!" << std::endl;
        std::cout << "  Thrusters: ";
        for (auto const& name : thruster_map.names)
            std::cout << name << " ";
        std::cout << std::endl << std::endl;

        // Test wrench conversion
        std::array<double, 3> wrench = { 100.0, 0.0, 0.0 };  // Pure surge
        std::vector<double> thrusts = thruster_map.wrench_to_thrusts(wrench);

        std::cout << "  Input wrench [surge, sway, yaw]: [" << wrench[0] << ", " << wrench[1] << ", " << wrench[2]
                  << "]" << std::endl;
        std::cout << "  Output thrusts:" << std::endl;
        for (size_t i = 0; i < thrusts.size(); ++i)
        {
            std::cout << "    " << names[i] << ": " << std::fixed << std::setprecision(4) << thrusts[i] << std::endl;
        }
        std::cout << std::endl;

        // Test with sway
        wrench = { 0.0, 50.0, 0.0 };  // Pure sway
        thrusts = thruster_map.wrench_to_thrusts(wrench);

        std::cout << "  Input wrench [surge, sway, yaw]: [" << wrench[0] << ", " << wrench[1] << ", " << wrench[2]
                  << "]" << std::endl;
        std::cout << "  Output thrusts:" << std::endl;
        for (size_t i = 0; i < thrusts.size(); ++i)
        {
            std::cout << "    " << names[i] << ": " << std::fixed << std::setprecision(4) << thrusts[i] << std::endl;
        }
        std::cout << std::endl;

        // Test with yaw
        wrench = { 0.0, 0.0, 25.0 };  // Pure yaw
        thrusts = thruster_map.wrench_to_thrusts(wrench);

        std::cout << "  Input wrench [surge, sway, yaw]: [" << wrench[0] << ", " << wrench[1] << ", " << wrench[2]
                  << "]" << std::endl;
        std::cout << "  Output thrusts:" << std::endl;
        for (size_t i = 0; i < thrusts.size(); ++i)
        {
            std::cout << "    " << names[i] << ": " << std::fixed << std::setprecision(4) << thrusts[i] << std::endl;
        }
    }
    catch (std::exception const& e)
    {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }

    std::cout << std::endl << "=== All Tests Completed Successfully ===" << std::endl;
    return 0;
}
