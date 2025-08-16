#pragma once

#include <Eigen/Dense>

#include "au/quantity.hh"
#include "au/units/meters.hh"
#include "mil_tools/os.hpp"

namespace mil::geometry
{

class Movement
{
    Eigen::Vector3d position_;

  public:
    Eigen::Vector3d position() const
    {
        return position_;
    }
    template <typename Unit, typename Precision>
    Movement forward(au::Quantity<Unit, Precision> distance)
    {
        Movement res = *this;
        res.position_.x() += distance.in(au::meters);
        return res;
    }
};

}  // namespace mil::geometry
