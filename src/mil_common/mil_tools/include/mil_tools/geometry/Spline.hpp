#pragma once

#include <cmath>

#include "mil_tools/geometry/Slerp.h"

namespace mil::geometry
{

template <size_t N>
class Spline
{
    std::array<Rotation, N> rotations_;
    std::array<Slerp, N> slerpq_;
    std::array<Slerp, N> slerpa_;
    size_t step_count_;

  public:
    Spline(std::array<Rotation, N> const& rotations) : rotations_(rotations)
    {
        std::array<Rotation, N> a;
        Rotation const& start = rotations_[0];
        Rotation const& end = rotations_[N - 1];
        a[0] = start;
        for (size_t i = 1; i < rotations.size(); i++)
        {
            T inv = rotations[i].inverse();
            a[i] = rotations[i] * start.inverse();
        }
    }

    Rotation at(double t) const
    {
        if (t < 0 || t > N)
        {
            throw std::out_of_range("t must be in [0, N]");
        }
        int m = static_cast<int>(std::floor(t));
        double ft = t - m;
        return Slerp
    }
}

};  // namespace mil::geometry
