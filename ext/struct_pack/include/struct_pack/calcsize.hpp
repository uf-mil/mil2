#pragma once
#include <tuple>
#include <utility>

#include "struct_pack/format.hpp"

namespace struct_pack {

// Implementation
template <typename Fmt>
constexpr auto calcsize(Fmt /*fmt*/) -> std::size_t {
    constexpr auto numItems = countItems(Fmt{});
    constexpr auto lastItem = getTypeOfItem<numItems - 1>(Fmt{});

    return getBinaryOffset<numItems - 1>(Fmt{}) + lastItem.size;
}

} // namespace struct_pack
