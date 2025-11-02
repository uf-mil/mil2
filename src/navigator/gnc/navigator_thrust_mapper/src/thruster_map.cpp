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

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigator_thrust_mapper
{

double vrx_force_to_command_scalar(double force)
{
    if (force > 250.0)
    {
        return 1.0;
    }
    else if (force < -100.0)
    {
        return -1.0;
    }
    else if (force > 3.27398)
    {
        // vrx inverse: force->command | force > 3.27398
        // -0.2 log(-0.246597 (0.56 - 4.73341/(-0.01 + x)^0.38))
        return -0.2 * std::log(-0.246597 * (0.56 - (4.73341 / std::pow((-0.01 + force), 0.38))));
    }
    else if (force < 0.0)
    {
        // vrx inverse: force->command | force < 3.27398
        // -0.113122 log(-154.285 (0.99 - (1.88948x10^12)/(199.13 + x)^5.34))
        return -0.113122 * std::log(-154.285 * (0.99 - ((1.88948e12) / std::pow((199.13 + force), 5.34))));
    }
    else
    {
        // approx broken range as straight line with 0.01cmd/3.2N
        return (0.01 / 3.27398) * force;
    }
}

Eigen::VectorXd vrx_force_to_command(Eigen::VectorXd const &forces)
{
    Eigen::VectorXd result(forces.size());
    for (int i = 0; i < forces.size(); ++i)
    {
        result(i) = vrx_force_to_command_scalar(forces(i));
    }
    return result;
}

std::function<Eigen::VectorXd(Eigen::VectorXd const &)> generate_linear_force_to_command(double ratio)
{
    return [ratio](Eigen::VectorXd const &force) { return force * ratio; };
}

ThrusterMap::ThrusterMap(std::vector<std::string> const &names, std::vector<std::array<double, 2>> const &positions,
                         std::vector<double> const &angles,
                         std::function<Eigen::VectorXd(Eigen::VectorXd const &)> const &force_to_command,
                         std::array<double, 2> const &force_limit, std::array<double, 2> const &com,
                         std::vector<std::string> const &joints)
  : names(names), joints(joints), force_to_command_(force_to_command), force_limit_(force_limit)
{
    // Validate force limits
    if (force_limit.size() != 2 || force_limit[1] > force_limit[0])
    {
        throw std::invalid_argument("force_limit must have 2 elements with force_limit[1] <= force_limit[0]");
    }

    size_t n_thrusters = names.size();
    if (positions.size() != n_thrusters || angles.size() != n_thrusters)
    {
        throw std::invalid_argument("names, positions, and angles must have the same length");
    }

    // Build thruster allocation matrix
    // Each column represents one thruster's contribution to [surge, sway, yaw]
    thruster_matrix_ = Eigen::MatrixXd::Zero(3, n_thrusters);

    for (size_t i = 0; i < n_thrusters; ++i)
    {
        // Offset from center of mass
        double l_x = positions[i][0] - com[0];
        double l_y = positions[i][1] - com[1];

        // Thruster direction
        double cos_angle = std::cos(angles[i]);
        double sin_angle = std::sin(angles[i]);

        // Torque effect: cross product of position and direction vectors
        double torque_effect = l_x * sin_angle - l_y * cos_angle;

        // Fill column i of the thruster matrix
        thruster_matrix_(0, i) = cos_angle;      // surge component
        thruster_matrix_(1, i) = sin_angle;      // sway component
        thruster_matrix_(2, i) = torque_effect;  // yaw component
    }

    // Compute pseudoinverse using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(thruster_matrix_, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Compute pseudoinverse manually: A^+ = V * Sigma^+ * U^T
    Eigen::VectorXd singularValues = svd.singularValues();
    double threshold = 1e-10 * std::max(thruster_matrix_.rows(), thruster_matrix_.cols()) * singularValues.maxCoeff();

    Eigen::VectorXd singularValues_inv(singularValues.size());
    for (int i = 0; i < singularValues.size(); ++i)
    {
        if (singularValues(i) > threshold)
        {
            singularValues_inv(i) = 1.0 / singularValues(i);
        }
        else
        {
            singularValues_inv(i) = 0.0;
        }
    }

    thruster_matrix_inv_ = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
}

ThrusterMap ThrusterMap::from_urdf(std::string const &urdf_string, std::string const &transmission_suffix)
{
    // TODO: Implement URDF parsing using urdfdom
    // For now, return a default instance with analytic 4-thruster configuration
    std::vector<std::string> names = { "FL", "FR", "BL", "BR" };
    std::vector<std::array<double, 2>> positions = { { 0.5, 0.5 }, { 0.5, -0.5 }, { -0.5, 0.5 }, { -0.5, -0.5 } };
    std::vector<double> angles = { 0.0, 0.0, 0.0, 0.0 };
    std::vector<std::string> joints = { "fl_joint", "fr_joint", "bl_joint", "br_joint" };

    auto force_to_command = generate_linear_force_to_command(1.0);
    std::array<double, 2> force_limit = { 250.0, -250.0 };

    return ThrusterMap(names, positions, angles, force_to_command, force_limit, { 0.0, 0.0 }, joints);
}

ThrusterMap ThrusterMap::from_vrx_urdf(std::string const &urdf_string)
{
    // TODO: Implement URDF parsing for VRX using urdfdom and tf2
    // For now, return a default instance with VRX-style force conversion
    std::vector<std::string> names = { "FL", "FR", "BL", "BR" };
    std::vector<std::array<double, 2>> positions = { { 0.5, 0.5 }, { 0.5, -0.5 }, { -0.5, 0.5 }, { -0.5, -0.5 } };
    std::vector<double> angles = { 0.0, 0.0, 0.0, 0.0 };

    auto force_to_command = [](Eigen::VectorXd const &forces) { return vrx_force_to_command(forces); };
    std::array<double, 2> force_limit = { 250.0, -100.0 };

    return ThrusterMap(names, positions, angles, force_to_command, force_limit);
}

std::vector<double> ThrusterMap::wrench_to_thrusts(std::array<double, 3> const &wrench) const
{
    // Convert wrench to Eigen vector
    Eigen::Vector3d wrench_vec(wrench[0], wrench[1], wrench[2]);

    // Solve: thruster_forces = thruster_matrix_inv * wrench
    Eigen::VectorXd forces = thruster_matrix_inv_ * wrench_vec;

    // Clip forces to limits
    for (int i = 0; i < forces.size(); ++i)
    {
        forces(i) = std::max(force_limit_[1], std::min(force_limit_[0], forces(i)));
    }

    // Convert forces to command units
    Eigen::VectorXd commands = force_to_command_(forces);

    // Convert to std::vector
    return std::vector<double>(commands.data(), commands.data() + commands.size());
}

}  // namespace navigator_thrust_mapper
