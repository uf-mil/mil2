#include <array>
#include <string_view>

template <size_t ArrSize>
constexpr bool operator==(std::array<char, ArrSize> const& arr, std::string_view str)
{
    // Assuming that str is a string literal
    if (ArrSize != str.size())
    {
        return false;
    }

    for (size_t i = 0; i < ArrSize; i++)
    {
        if (arr[i] != str[i])
        {
            return false;
        }
    }

    return true;
}

template <typename F, typename S, typename F2, typename S2>
constexpr bool operator==(std::pair<F, S> const& lhs, std::pair<F2, S2> const& rhs)
{
    return (lhs.first == static_cast<F>(rhs.first)) && (lhs.second == static_cast<S>(rhs.second));
}

// Crazy workaround because std::string_view.operator== isn't constexpr...
constexpr bool equals(std::string_view const lhs, std::string_view const rhs)
{
    if (lhs.size() != rhs.size())
    {
        return false;
    }

    for (size_t i = 0; i < lhs.size(); i++)
    {
        if (lhs[i] != rhs[i])
        {
            return false;
        }
    }

    return true;
}

template <typename Lhs, typename Rhs>
constexpr bool equals(Lhs const& lhs, Rhs const& rhs)
{
    return lhs == rhs;
}

template <typename... Lhs, typename... Rhs, size_t... Is>
constexpr bool equals(std::tuple<Lhs...> const& lhs, std::tuple<Rhs...> const& rhs, std::index_sequence<Is...>)
{
    bool res[] = { equals(std::get<Is>(lhs), std::get<Is>(rhs))... };
    for (bool b : res)
    {
        if (!b)
        {
            return false;
        }
    }

    return true;
}

template <typename... Lhs, typename... Rhs>
constexpr bool equals(std::tuple<Lhs...> const& lhs, std::tuple<Rhs...> const& rhs)
{
    if constexpr (sizeof...(Lhs) != sizeof...(Rhs))
    {
        return false;
    }

    return equals(lhs, rhs, std::make_index_sequence<sizeof...(Lhs)>());
}
