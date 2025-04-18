#pragma once

#include <algorithm>
#include <cfloat>
#include <numeric>

namespace mil_tools::math
{

template <typename T>
bool is_close(T const& a, T const& b, T epsilon = FLT_EPSILON)
{
    // standard algorithm does not work when one of values is zero
    // but basic algorithm should work
    if (a == 0 || b == 0)
    {
        return std::abs(a - b) <= epsilon;
    }
    float diff = std::abs(a - b);
    float max_val = std::max(std::abs(a), std::abs(b));
    return (diff <= max_val * epsilon);
}

}  // namespace mil_tools::math
