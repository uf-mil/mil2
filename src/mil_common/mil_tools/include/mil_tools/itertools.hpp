#pragma once

#include <tuple>
#include <iterator>

namespace mil_tools::itertools
{
template <typename T,
    typename TIter = decltype(std::begin(std::declval<T>())),
    typename = decltype(std::end(std::declval<T>()))>
constexpr auto enumerate(T && iterable);
}  // namespace mil_tools::itertools
