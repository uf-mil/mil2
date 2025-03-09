#include "constexpr_compare.hpp"
#include "constexpr_require.hpp"
#include "struct_pack.hpp"

#include <catch2/catch.hpp>

TEST_CASE("consume string", "[struct_pack::string]") {
    // REQUIRE_STATIC(struct_pack::detail::consumeNumber("123", 0)
    //                == std::make_pair(123, 3));
    // REQUIRE_STATIC(struct_pack::detail::consumeNumber("c", 0)
    //                == std::make_pair(0, 0));
    // REQUIRE_STATIC(struct_pack::detail::consumeNumber("c12345c", 1)
    //                == std::make_pair(12345, 6));
    //
    // REQUIRE_STATIC(struct_pack::detail::consumeNumber("c12345c", 0)
    //                == std::make_pair(0, 0));
}
