#pragma once

// https://www.reddit.com/r/cpp/comments/101ktve/c20_string_literal_operator_template_useful/
#include <algorithm>
#include <string_view>

template <std::size_t N>
struct string_container {
    constexpr string_container(const char (&s)[N]) {
        std::copy_n(s, N, data);
    }

    constexpr auto size() const -> std::size_t {
        return N - 1;
    }

    char data[N]{};
};

template <auto container>
struct type_string {
    static constexpr auto data() -> const char * {
        return container.data;
    }
    static constexpr auto size() -> std::size_t {
        return container.size();
    }
    static constexpr auto view() -> std::string_view {
        return std::string_view{data(), size()};
    }
};

template <string_container container>
constexpr auto operator""_fmt() {
    return type_string<container>{};
}
