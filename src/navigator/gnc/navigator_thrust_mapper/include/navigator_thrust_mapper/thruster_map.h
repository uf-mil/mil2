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

#ifndef NAVIGATOR_THRUST_MAPPER_THRUSTER_MAP_H
#define NAVIGATOR_THRUST_MAPPER_THRUSTER_MAP_H

#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>

namespace navigator_thrust_mapper
{

/// Convert force to command scalar for VRX simulation
/// Implements the inverse thrust dynamics model for VRX
double vrx_force_to_command_scalar(double force);

/// Vectorized version of vrx_force_to_command_scalar
Eigen::VectorXd vrx_force_to_command(Eigen::VectorXd const &forces);

/// Generate a linear force-to-command conversion function with given ratio
std::function<Eigen::VectorXd(Eigen::VectorXd const &)> generate_linear_force_to_command(double ratio);

/// Helper class to map between body forces/torques and thruster outputs
/// Implements least-squares thrust allocation as described in:
/// Christiaan De With "Optimal Thrust Allocation Methods for Dynamic Positioning of Ships"
class ThrusterMap
{
  public:
    /// Default constructor (creates empty/invalid ThrusterMap)
    ThrusterMap() = default;

    /// Constructor for ThrusterMap
    /// @param names List of thruster names
    /// @param positions List of (x, y) positions for each thruster in meters
    /// @param angles List of angles for each thruster in radians
    /// @param force_to_command Function to convert forces to command units
    /// @param force_limit Tuple (MAX_FORWARD, MAX_REVERSE) maximum force in newtons
    /// @param com Center of mass offset from base_link, defaults to (0, 0)
    /// @param joints Joint names corresponding to each thruster
    ThrusterMap(std::vector<std::string> const &names, std::vector<std::array<double, 2>> const &positions,
                std::vector<double> const &angles,
                std::function<Eigen::VectorXd(Eigen::VectorXd const &)> const &force_to_command,
                std::array<double, 2> const &force_limit, std::array<double, 2> const &com = { 0.0, 0.0 },
                std::vector<std::string> const &joints = {});

    /// Create a ThrusterMap from a URDF string
    /// Expects each thruster to be connected to a transmission ending in "_thruster_transmission"
    /// @param urdf_string URDF XML as a string
    /// @param transmission_suffix Suffix to identify thruster transmissions
    static ThrusterMap from_urdf(std::string const &urdf_string, std::string const &transmission_suffix = "_thruster_"
                                                                                                          "transmissio"
                                                                                                          "n");

    /// Create a ThrusterMap for VRX-style naming and force conversions
    /// @param urdf_string URDF XML as a string
    static ThrusterMap from_vrx_urdf(std::string const &urdf_string);

    /// Convert body wrench to individual thruster thrusts using least-squares allocation
    /// @param wrench Array of [surge, sway, yaw] forces/torques in Newtons/N*m
    /// @return Vector of thruster efforts in command units
    std::vector<double> wrench_to_thrusts(std::array<double, 3> const &wrench) const;

    // Public members for quick access
    std::vector<std::string> names;
    std::vector<std::string> joints;

  private:
    std::function<Eigen::VectorXd(Eigen::VectorXd const &)> force_to_command_;
    std::array<double, 2> force_limit_;
    Eigen::MatrixXd thruster_matrix_;
    Eigen::MatrixXd thruster_matrix_inv_;
};

}  // namespace navigator_thrust_mapper

#endif  // NAVIGATOR_THRUST_MAPPER_THRUSTER_MAP_H
