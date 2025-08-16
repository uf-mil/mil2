#include "au/quantity.hh"
#include "mil_tools/geometry/Movement.hpp"

namespace mil::geometry
{

template <typename Unit, typename Precision>
constexpr Movement forward(au::Quantity<Unit, Precision> distance)
{
    return Movement{}.forward(distance);
}

}  // namespace mil::geometry
