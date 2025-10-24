#pragma once

#include <iterator>
#include <tuple>

namespace mil::itertools
{
// https://www.reedbeta.com/blog/python-like-enumerate-in-cpp17/
template <typename T, typename TIter = decltype(std::begin(std::declval<T>())),
          typename = decltype(std::end(std::declval<T>()))>
constexpr auto enumerate(T&& iterable)
{
    struct iterator
    {
        size_t i;
        TIter iter;
        bool operator!=(iterator const& other) const
        {
            return iter != other.iter;
        }
        void operator++()
        {
            ++i;
            ++iter;
        }
        // using std::tie over std::make_tuple to prevent copies
        auto operator*() const
        {
            return std::tie(i, *iter);
        }
    };
    struct iterable_wrapper
    {
        // changed to T& to prevent copies
        T& iterable;
        auto begin()
        {
            return iterator{ 0, std::begin(iterable) };
        }
        auto end()
        {
            return iterator{ 0, std::end(iterable) };
        }
    };
    return iterable_wrapper{ std::forward<T>(iterable) };
}
}  // namespace mil::itertools
