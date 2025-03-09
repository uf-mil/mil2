#pragma once
#include <iterator>
#include <utility>

namespace struct_pack::detail {

struct format_string {};

constexpr bool isDigit(char ch) {
    return ch >= '0' && ch <= '9';
}

template <size_t Size>
constexpr std::pair<size_t, size_t> consumeNumber(const char (&str)[Size],
                                                  size_t offset) {
    size_t num = 0;
    size_t i = offset;
    for (; isDigit(str[i]) && i < Size; i++) {
        num = static_cast<size_t>(num * 10 + (str[i] - '0'));
    }

    return {num, i};
}

} // namespace struct_pack::detail

#define PY_STRING(s)                                                           \
    [] {                                                                       \
        struct S : struct_pack::detail::format_string {                        \
            static constexpr auto value() -> decltype(auto) {                  \
                return s;                                                      \
            }                                                                  \
            static constexpr auto size() -> size_t {                           \
                return std::size(value()) - 1;                                 \
            }                                                                  \
            static constexpr auto at(size_t i) {                               \
                return value()[i];                                             \
            };                                                                 \
        };                                                                     \
        return S{};                                                            \
    }()
