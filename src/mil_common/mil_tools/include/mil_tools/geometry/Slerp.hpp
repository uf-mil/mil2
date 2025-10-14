#pragma once

#include <cmath>

#include "mil_tools/geometry/Rotation.hpp"

namespace mil::geometry
{

class Slerp
{
  private:
    Rotation left, right;
    double ln, atan2;
    bool equal;

  public:
    Slerp(Rotation const& r1, Rotation const& r2)
    {
        left = r1;
        right = r2;

        // left.normalize();
        // right.normalize();

        equal = r1 == r2;
        /*
        if (equal)
            return;

        left.conjugate();
        right = left * right;
        left.conjugate();

        double abs_imag = right.abs_imag();
        imag = right.imaginary() / abs_imag;

        ln = 0.5 * std::log(right.real() * right.real() + abs_imag * abs_imag);
        double real_val = right.real();
        atan2 = std::atan2(abs_imag, real_val);
        */
    }
    Rotation at(double t) const
    {
        if (equal)
            return left;
        return left.quat().slerp(t, right.quat());
    }
};

};  // namespace mil::geometry
