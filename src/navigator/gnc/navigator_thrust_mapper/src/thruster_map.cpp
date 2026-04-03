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

ThrusterMap::ThrusterMap(std::vector<std::string> const &names, std::vector<std::array<double, 2>> const &positions,
                         std::vector<double> const &angles, std::array<double, 2> const &force_limit,
                         std::array<double, 2> const &com)

  : names(names), force_limit_(force_limit)
{
    if (force_limit.size() != 2 || force_limit[1] > force_limit[0])
        throw std::invalid_argument("force_limit invalid");

    size_t n = names.size();
    if (positions.size() != n || angles.size() != n)
        throw std::invalid_argument("size mismatch");

    thruster_matrix_ = Eigen::MatrixXd::Zero(3, n);
    for (size_t i = 0; i < n; ++i)
    {
        double l_x = positions[i][0] - com[0];
        double l_y = positions[i][1] - com[1];
        double cos_a = std::cos(angles[i]);
        double sin_a = std::sin(angles[i]);
        double torque = l_x * sin_a - l_y * cos_a;
        thruster_matrix_(0, i) = cos_a;
        thruster_matrix_(1, i) = sin_a;
        thruster_matrix_(2, i) = torque;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(thruster_matrix_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd sv = svd.singularValues();
    double thresh = 1e-10 * std::max(thruster_matrix_.rows(), thruster_matrix_.cols()) * sv.maxCoeff();

    Eigen::VectorXd sv_inv(sv.size());
    for (int i = 0; i < sv.size(); ++i)
        sv_inv(i) = (sv(i) > thresh) ? 1.0 / sv(i) : 0.0;

    thruster_matrix_inv_ = svd.matrixV() * sv_inv.asDiagonal() * svd.matrixU().transpose();
}

std::vector<double> ThrusterMap::wrench_to_thrusts(std::array<double, 3> const &wrench) const
{
    Eigen::Vector3d w(wrench[0], wrench[1], wrench[2]);
    Eigen::VectorXd forces = thruster_matrix_inv_ * w;

    for (int i = 0; i < forces.size(); ++i)
        forces(i) = std::max(force_limit_[1], std::min(force_limit_[0], forces(i)));

    return std::vector<double>(forces.data(), forces.data() + forces.size());
}

}  // namespace navigator_thrust_mapper
