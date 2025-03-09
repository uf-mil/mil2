#pragma once

#include <string_view>
#include <tuple>

#include "struct_pack/data_view.hpp"
#include "struct_pack/debug.hpp"
#include "struct_pack/new_format.hpp"
#include "struct_pack/print.hpp"

namespace struct_pack::detail {
template <auto container>
struct fmt_string {
    static constexpr auto data() -> const char * {
        return container.data;
    }

    static constexpr auto size() -> std::size_t {
        return container.size();
    }

    static constexpr auto view() -> std::string_view {
        return std::string_view{data(), size()};
    }

    static constexpr auto at(std::size_t i) -> char {
        // static_assert(i < size());
        return data()[i];
    }

    static constexpr auto consume_number(size_t offset)
        -> std::pair<size_t, size_t> {
        size_t num = 0;
        size_t i = offset;
        for (; is_digit(at(i)) && i < container.size(); i++) {
            num = static_cast<size_t>(num * 10 + (at(i) - '0'));
        }

        return {num, i};
    }

    static constexpr auto count_items() -> size_t {
        size_t item_count = 0;
        size_t num = 1;
        for (size_t i = 0; i < size(); i++) {
            auto current = at(i);
            if (i == 0 && is_format_mode(current)) {
                continue;
            }

            if (is_digit(current)) {
                std::tie(num, i) = consume_number(i);
                i--; // to combat the i++ in the loop
                continue;
            }

            if (current == 's') {
                item_count++;
            } else {
                item_count += num;
            }
            num = 1;
        }

        return item_count;
    }

    static constexpr auto format_mode() {
        if constexpr (is_format_mode(at(0))) {
            constexpr auto first_char = at(0);
            return new_FormatMode<first_char>{};
        } else {
            return new_FormatMode<'@'>{};
        }
    }

    struct RawFormatType {
        char           format_char;
        size_t         repeat;
        constexpr auto is_string() const -> bool {
            return format_char == 's';
        }
    };

    struct FormatType {
        char           format_char;
        size_t         format_size;
        size_t         size;
        constexpr auto is_string() const -> bool {
            return format_char == 's';
        }

        constexpr auto need_align() const -> bool {
            return format_size > 1;
        }
    };

    template <char FormatChar, size_t Repeat = 1>
    static constexpr auto get_size() -> size_t {
        if constexpr (format_mode().is_native()) {
            return BigEndianFormat<FormatChar>::native_size() * Repeat;
        } else {
            return BigEndianFormat<FormatChar>::size() * Repeat;
        }
    }

    template <size_t Item, size_t N>
    static constexpr auto unwrapped_item(RawFormatType (&formats)[N])
        -> RawFormatType {
        size_t current_item = 0;
        for (size_t i = 0; i < N; i++) {
            for (size_t repeat = 0; repeat < formats[i].repeat; repeat++) {
                auto current_type = formats[i];
                if (current_item == Item) {
                    if (!current_type.is_string()) {
                        current_type.repeat = 1;
                    }
                    return current_type;
                }
                current_item++;
                if (current_type.is_string()) {
                    break;
                }
            }
        }
        // cannot get here, Item < ArrSize
        return {0, 0};
    }

    template <size_t Index, size_t... Items>
    static constexpr auto
    type_of_item_helper(std::index_sequence<Items...> /*unused*/)
        -> RawFormatType {
        constexpr auto format_string = std::array{at(Items)...};
        RawFormatType  wrapped_types[count_items()]{};
        size_t         current_type = 0;
        for (size_t i = 0; i < sizeof...(Items); i++) {
            if (is_format_mode(format_string[i])) {
                continue;
            }
            auto repeat_count = consume_number(i);
            i = repeat_count.second;
            wrapped_types[current_type].format_char = format_string[i];
            wrapped_types[current_type].repeat = repeat_count.first;
            if (repeat_count.first == 0) {
                wrapped_types[current_type].repeat = 1;
            }
            current_type++;
        }
        return unwrapped_item<Index>(wrapped_types);
    }

    template <size_t Index>
    static constexpr auto type_of_item() -> FormatType {
        static_assert(Index < count_items(),
                      "Item requested must be inside the format");
        constexpr auto format = RawFormatType{
            type_of_item_helper<Index>(std::make_index_sequence<size()>())};
        constexpr auto type
            = FormatType{format.format_char,
                         get_size<format.format_char>(),
                         get_size<format.format_char, format.repeat>()};
        return type;
    }

    template <size_t... Items>
    static constexpr auto
    binary_offset_helper(std::index_sequence<Items...> /*unused*/) -> size_t {
        constexpr auto item_types = std::array{type_of_item<Items>()...};
        constexpr auto mode = format_mode();
        size_t         size = 0;
        for (size_t i = 0; i < sizeof...(Items) - 1; i++) {
            size += item_types[i].size;
            if (mode.should_pad()) {
                if (item_types[i + 1].need_align()) {
                    auto current_alignment
                        = (size % item_types[i + 1].format_size);
                    if (current_alignment != 0) {
                        size += item_types[i + 1].format_size
                                - current_alignment;
                    }
                }
            }
        }
        return size;
    }
    template <size_t Index>
    static constexpr auto binary_offset() -> size_t {
        return binary_offset_helper(std::make_index_sequence<Index + 1>());
    }

    // https://docs.python.org/3/library/struct.html#struct.calcsize
    static constexpr auto calcsize() -> std::size_t {
        // 3H4B10s:
        // count_items: 3*4 + 4 * 1 + 10 = 16
        // with padding: 3 * 4 + 4*1 + (4bytes) + 10 = 20
        constexpr auto num_items = count_items();
        constexpr auto last_item = type_of_item<num_items - 1>();
        return binary_offset<num_items - 1>() + last_item.size;
    }

    template <typename RepType>
    static constexpr auto
    pack_element(char *data, bool big_endian, FormatType format, RepType elem) {
        if constexpr (std::is_same_v<RepType, std::string_view>) {
            // Trim the string size to the repeat count specified in the
            // format
            elem = std::string_view{elem.data(),
                                    std::min(elem.size(), format.size)};
            // } else {
            //     (void) format; // Unreferenced if constexpr RepType !=
            //     string_view
        }
        auto view = data_view<char>{data, big_endian};
        PRINT("pack store: {} <- {}", (void *) data, elem);
        data::store(view, elem);
    }

    template <typename RepType, typename T>
    static constexpr auto convert_to(const T &val) -> RepType {
        // If T is char[], and RepType is string_view - construct directly
        // with std::size(val)
        //  because std::string_view doesn't have a constructor taking a
        //  char(&)[]
        if constexpr (std::is_array_v<T>
                      && std::is_same_v<std::remove_extent_t<T>, char>
                      && std::is_same_v<RepType, std::string_view>) {
            return RepType(std::data(val), std::size(val));
        } else {
            return static_cast<RepType>(val);
        }
    }

    template <size_t... Items, typename... Args>
    static auto pack(std::index_sequence<Items...> /*unused*/, Args &&...args) {
        constexpr auto mode = format_mode();
        PRINT("format mode: {}", mode.format());

        constexpr auto num_bytes = calcsize();
        PRINT("calcsize: {}", num_bytes);

        constexpr auto formats = std::array{type_of_item<Items>()...};
        using Types = std::tuple<
            RepresentedType<decltype(mode), formats[Items].format_char>...>;

        // Convert args to a tuple of the represented types
        Types types
            = std::make_tuple(convert_to<std::tuple_element_t<Items, Types>>(
                std::forward<Args>(args))...);
        constexpr auto offsets = std::array{binary_offset<Items>()...};
        PRINT("offset: {}", print_hpp::P(offsets));

        auto output = std::array<char, num_bytes>{};
        (pack_element(output.data() + offsets[Items],
                      mode.is_big_endian(),
                      formats[Items],
                      std::get<Items>(types)),
         ...);
        return output;
    }
};
} // namespace struct_pack::detail
