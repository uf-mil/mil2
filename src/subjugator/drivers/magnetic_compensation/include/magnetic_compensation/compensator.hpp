#pragma once

#include <Eigen/Dense>

namespace mil::magnetic_compensation
{

// Apply the hard/soft-iron correction: undo the offset, then the distortion.
inline Eigen::Vector3d compensate(Eigen::Matrix3d const &scale_inverse, Eigen::Vector3d const &shift,
                                  Eigen::Vector3d const &raw)
{
    return scale_inverse * (raw - shift);
}

}  // namespace mil::magnetic_compensation
