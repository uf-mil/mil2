#pragma once

#include <iostream>

#include <Eigen/Dense>

#include "au/conversion_policy.hh"
#include "au/quantity.hh"
#include "au/units/meters.hh"
#include "au/units/radians.hh"
#include "mil_tools/geometry/Rotation.hpp"

namespace mil::geometry
{

struct Movement
{
    Eigen::Vector3d position{ 0, 0, 0 };
    Rotation orientation{ 0, 0, 0, 1 };

    template <typename Unit, typename Precision>
    Movement forward(au::Quantity<Unit, Precision> distance)
    {
        Movement res = *this;
        res.position.x() += distance.in(au::meters);
        return res;
    }
    template <typename Unit, typename Precision>
    Movement backward(au::Quantity<Unit, Precision> distance)
    {
        Movement res = *this;
        res.position.x() -= distance.in(au::meters);
        return res;
    }
    template <typename Unit, typename Precision>
    Movement right(au::Quantity<Unit, Precision> distance)
    {
        Movement res = *this;
        res.position.y() += distance.in(au::meters);
        return res;
    }
    template <typename Unit, typename Precision>
    Movement left(au::Quantity<Unit, Precision> distance)
    {
        Movement res = *this;
        std::cout << "LEFT y() before " << res.position.y() << "\n";
        res.position.y() -= distance.in(au::meters);
        std::cout << "LEFT y() after " << res.position.y() << "\n";
        return res;
    }
    template <typename Unit, typename Precision>
    Movement up(au::Quantity<Unit, Precision> distance)
    {
        Movement res = *this;
        res.position.z() += distance.in(au::meters);
        return res;
    }
    template <typename Unit, typename Precision>
    Movement down(au::Quantity<Unit, Precision> distance)
    {
        Movement res = *this;
        res.position.z() -= distance.in(au::meters);
        return res;
    }
    template <typename Unit, typename Precision>
    Movement yaw_right(au::Quantity<Unit, Precision> angle)
    {
        Movement res = *this;
        res.orientation = res.orientation * Rotation({ 0, 0, angle.in(au::radians) });
        return res;
    }
    template <typename Unit, typename Precision>
    Movement yaw_left(au::Quantity<Unit, Precision> angle)
    {
        Movement res = *this;
        res.orientation = res.orientation * Rotation({ 0, 0, -angle.in(au::radians) });
        return res;
    }

    bool operator==(Movement const& other) const
    {
        return position == other.position && orientation == other.orientation;
    }

    // output and formatting
    friend std::ostream& operator<<(std::ostream& os, Movement const& movement)
    {
        os << "Movement(position: ("
           << "x: " << movement.position.x() << ", "
           << "y: " << movement.position.y() << ", "
           << "z: " << movement.position.z() << "), "
           << "orientation: " << movement.orientation << ")";
        return os;
    }
};

}  // namespace mil::geometry
