#include <catch.hpp>

#include "constexpr_compare.h"
#include "constexpr_require.h"
#include "cppystruct.h"

TEST_CASE("consume string", "[cppystruct::string]")
{
    REQUIRE_STATIC(pystruct::internal::consumeNumber("123", 0) == std::make_pair(123, 3));
    REQUIRE_STATIC(pystruct::internal::consumeNumber("c", 0) == std::make_pair(0, 0));
    REQUIRE_STATIC(pystruct::internal::consumeNumber("c12345c", 1) == std::make_pair(12345, 6));

    REQUIRE_STATIC(pystruct::internal::consumeNumber("c12345c", 0) == std::make_pair(0, 0));
}
