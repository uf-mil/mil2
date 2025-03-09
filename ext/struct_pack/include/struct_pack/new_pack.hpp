#include "struct_pack/string_fmt.hpp"
#include "struct_pack/string_literal.hpp"

namespace struct_pack {

template <string_container container, typename... Args>
auto new_pack(Args &&...args) {
    using Fmt = detail::fmt_string<container>;
    constexpr size_t N = Fmt::count_items();
    static_assert(N == sizeof...(args), "Parameter number does not match");

    return Fmt::pack(std::make_index_sequence<N>{},
                     std::forward<Args>(args)...);
}
} // namespace struct_pack
