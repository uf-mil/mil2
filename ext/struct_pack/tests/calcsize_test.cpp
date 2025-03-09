#include <struct_pack.hpp>

#include <catch2/catch.hpp>

#define REQUIRE_STATIC(x)                                                      \
    REQUIRE(x);                                                                \
    static_assert(x, #x);

TEST_CASE("no count, native byte order (with padding)",
          "[struct_pack::calcsize]") {
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("c")) == 1);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("cc")) == 2);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("cch")) == 4);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("cchh")) == 6);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("cchH")) == 6);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("cchHi")) == 12);
}

TEST_CASE("count", "[struct_pack::calcsize]") {
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("4c")) == 4);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("40c")) == 40);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("c4c")) == 5);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("2i4c3h")) == 18);
}

TEST_CASE("byte order", "[struct_pack::calcsize]") {
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("!ci")) == 5);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING(">ci")) == 5);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("<ci")) == 5);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("=ci")) == 5);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("@ci")) == 8);
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("ci")) == 8);

    // Padding sanity
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("ic")) == 5);
}

TEST_CASE("count with padding", "[struct_pack::calcsize]") {
    REQUIRE_STATIC(struct_pack::calcsize(PY_STRING("c4ci")) == 12);
}
