#include "struct_pack.hpp"

auto main() -> int {
    static_assert(std::same_as<decltype("123"_fmt), decltype("123"_fmt)>);
    static_assert(not std::same_as<decltype("123"_fmt), decltype("456"_fmt)>);
    {
        constexpr auto a = "hello"_fmt;
        constexpr auto b = "hello"_fmt;
        static_assert(a.view() == b.view());
    }
    {
        constexpr auto a = "hello"_fmt;
        constexpr auto b = "goodbye"_fmt;
        static_assert(a.view() != b.view());
    }
}
