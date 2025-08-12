#pragma once

#include <cmath>

#include "mil_tools/geometry/Rotation.hpp"

namespace mil_tools::geometry
{

class Slerp
{
  private:
    Rotation left, right, imag;
    double ln, atan2;
    bool equal;

  public:
    Slerp(Rotation const& r1, Rotation const& r2)
    {
        left = r1;
        right = r2;

        left.normalize();
        right.normalize();
    }
    Rotation at(double t) const
    {
        return left.quat_.slerp(t, right.quat_);
    }
};

};  // namespace mil_tools::geometry
