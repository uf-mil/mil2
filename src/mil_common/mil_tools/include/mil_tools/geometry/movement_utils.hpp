#include "au/quantity.hh"
#include "au/units/meters.hh"
#include "au/units/radians.hh"
#include "mil_tools/geometry/Movement.hpp"

namespace mil::geometry
{

template <typename Unit, typename Precision>
Movement forward(au::Quantity<Unit, Precision> distance)
{
    return Movement{}.forward(distance);
}

template <typename Unit, typename Precision>
Movement backward(au::Quantity<Unit, Precision> distance)
{
    return Movement{}.backward(distance);
}

template <typename Unit, typename Precision>
Movement left(au::Quantity<Unit, Precision> distance)
{
    return Movement{}.left(distance);
}

template <typename Unit, typename Precision>
Movement right(au::Quantity<Unit, Precision> distance)
{
    return Movement{}.right(distance);
}

template <typename Unit, typename Precision>
Movement up(au::Quantity<Unit, Precision> distance)
{
    return Movement{}.up(distance);
}

template <typename Unit, typename Precision>
Movement down(au::Quantity<Unit, Precision> distance)
{
    return Movement{}.down(distance);
}

template <typename Unit, typename Precision>
Movement yaw_right(au::Quantity<Unit, Precision> angle)
{
    return Movement{}.yaw_right(angle);
}

template <typename Unit, typename Precision>
Movement yaw_left(au::Quantity<Unit, Precision> angle)
{
    return Movement{}.yaw_left(angle);
}

}  // namespace mil::geometry
