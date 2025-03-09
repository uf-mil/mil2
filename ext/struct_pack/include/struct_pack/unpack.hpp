#pragma once
#include <array>

#include "struct_pack/calcsize.hpp"
#include "struct_pack/data_view.hpp"

namespace struct_pack {

template <typename Fmt, typename Input>
constexpr auto unpack(Fmt, Input &&packedInput);

namespace detail {

    template <typename Fmt, size_t... Items, typename Input>
    constexpr auto unpack(std::index_sequence<Items...>, Input &&packedInput);

} // namespace detail

template <typename Fmt, typename Input>
constexpr auto unpack(Fmt, Input &&packedInput) {
    return detail::unpack<Fmt>(std::make_index_sequence<countItems(Fmt{})>(),
                               std::forward<Input>(packedInput));
}

template <size_t Item, typename UnpackedType>
constexpr auto unpackElement(const char *begin, size_t size, bool bigEndian) {
    data_view<const char> view(begin, bigEndian);
    view.size = size;

    return data::get<UnpackedType>(view);
}

template <typename Fmt, size_t... Items, typename Input>
constexpr auto detail::unpack(std::index_sequence<Items...>,
                              Input &&packedInput) {
    constexpr auto formatMode = struct_pack::getFormatMode(Fmt{});

    constexpr FormatType formats[]
        = {struct_pack::getTypeOfItem<Items>(Fmt{})...};

    using Types = std::tuple<
        typename struct_pack::RepresentedType<decltype(formatMode),
                                              formats[Items].formatChar>...>;

    constexpr size_t offsets[] = {getBinaryOffset<Items>(Fmt{})...};
    auto             unpacked = std::make_tuple(
        unpackElement<Items, std::tuple_element_t<Items, Types>>(
            std::data(packedInput) + offsets[Items],
            formats[Items].size,
            formatMode.isBigEndian())...);

    return unpacked;
}

} // namespace struct_pack
