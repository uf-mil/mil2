#pragma once

#include <iostream>
#include <optional>
#include <ostream>
#include <ranges>
#include <sstream>
#include <type_traits>
#include <variant>

namespace print_custom {
template <typename T>
struct printer {
    printer() = delete;
    printer(const printer &) = delete;
    auto operator=(const printer &) = delete;
};

template <typename T>
inline auto print_to(std::ostream &out, T &&t);
} // namespace print_custom

namespace print_detail {

template <typename T>
constexpr auto name_of() -> std::string_view {
    std::string_view sv;
#if __clang__
    sv = __PRETTY_FUNCTION__;
    sv = sv.substr(sv.find_last_of('=') + 2);
    sv = sv.substr(0, sv.size() - 1);
#elif __GNUC__
    sv = __PRETTY_FUNCTION__;
    sv = sv.substr(sv.find_first_of('=') + 2);
    sv = sv.substr(0, sv.find_first_of(';'));
#else
#error "name_of not implemented on this platform"
#endif
    return sv;
}

template <typename Container>
concept is_iterable_v = requires(Container &c) {
    std::ranges::begin(c);
    std::ranges::end(c);
};

template <typename Container>
concept is_mappable_v = requires(Container &c) {
    typename Container::key_type;
    typename Container::mapped_type;
    {
        c[std::declval<const typename Container::key_type &>()]
    } -> std::same_as<typename Container::mapped_type &>;
};

template <typename T>
concept is_variant_v = requires { std::variant_size<T>::value; };

template <typename T>
concept is_optional_v = requires(T &t) {
    { t.has_value() } -> std::same_as<bool>;
    { t.value() } -> std::convertible_to<T>;
};

template <typename Container>
concept is_tuple_v = requires { std::tuple_size<Container>::value; };

template <typename T>
concept has_printer_print_v = requires(print_custom::printer<T> &printer,
                                       const T                  &t,
                                       std::ostream             &out) {
    { printer.print(t, out) };
};

template <typename T>
auto print_to(std::ostream &out, const T &t) {
    if constexpr (std::is_convertible_v<T, std::string_view>) {
        out << '"' << t << '"';
    } else if constexpr ((std::is_integral_v<T> && !std::is_same_v<T, bool>)
                         || std::is_floating_point_v<T>) {
        out << t;
    } else if constexpr (std::is_same_v<T, bool>) {
        out << (t ? "true" : "false");
    } else if constexpr (is_mappable_v<T>) {
        out << '{';
        auto first = true;
        for (const auto &elem : t) {
            if (!first) {
                out << ", ";
            }
            first = false;
            print_to(out, elem.first);
            out << ": ";
            print_to(out, elem.second);
        }
        out << "}";
    } else if constexpr (is_iterable_v<T>) {
        out << '[';
        auto first = true;
        for (const auto &elem : t) {
            if (!first) {
                out << ", ";
            }
            first = false;
            print_to(out, elem);
        }
        out << "]";
    } else if constexpr (is_variant_v<T>) {
        std::visit(
            [&out](const auto &value) {
                print_to(out, value);
            },
            t);
    } else if constexpr (is_optional_v<T>) {
        if (t) {
            print_to(out, t.value());
        } else {
            out << "nullopt";
        }
    } else if constexpr (std::is_same_v<T, std::nullopt_t>) {
        out << "nullopt";
    } else if constexpr (is_tuple_v<T>) {
        out << '(';
        auto first = true;
        std::apply(
            [&out, &first](auto &&...args) {
                (([&out, &first, &args] {
                     if (!first) {
                         out << ", ";
                     }
                     print_to(out, std::forward<decltype(args)>(args));
                     first = false;
                 }()),
                 ...);
            },
            t);
        out << ')';
    } else if constexpr (has_printer_print_v<T>) {
        print_custom::printer<T>().print(t, out);
    } else {
        out << "unprintable type " << name_of<T>();
    }
}

} // namespace print_detail

template <typename T>
auto print_custom::print_to(std::ostream &out, T &&t) {
    print_detail::print_to(out, std::forward<T>(t));
}

namespace print_hpp {
template <typename T>
auto print(T &&t) {
    print_detail::print_to(std::cout, std::forward<T>(t));
    std::cout << '\n';
}

template <typename T, typename... Ts>
void print(T &&t, Ts &&...ts) {
    print_detail::print_to(std::cout, std::forward<T>(t));
    ((std::cout << ' ',
      print_detail::print_to(std::cout, std::forward<Ts>(ts))),
     ...);
    std::cout << '\n';
}

// pretty print
template <typename T>
auto P(T &&t) {
    std::ostringstream oss; // 千万别用 `std::ostrstream` !
    print_detail::print_to(oss, std::forward<T>(t));
    return oss.str();
}
} // namespace print_hpp
