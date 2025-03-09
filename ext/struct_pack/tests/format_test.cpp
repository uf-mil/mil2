#include "constexpr_require.hpp"
#include "struct_pack.hpp"

#define CATCH_CONFIG_ENABLE_TUPLE_STRINGMAKER
#include <catch2/catch.hpp>

TEST_CASE("item count without format-count", "[struct_pack::format]") {
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("c")) == 1);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("cc")) == 2);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("cch")) == 3);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("cchh")) == 4);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("cchH")) == 4);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("cchHi")) == 5);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("cchHis")) == 6);
}

TEST_CASE("item count with format-count", "[struct_pack::format]") {
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("4c")) == 4);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("40c")) == 40);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("c4c")) == 5);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("2i4c3h")) == 9);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("c4ci")) == 6);

    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("4s")) == 1);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("3c4s")) == 4);
}

TEST_CASE("item count with byte order", "[struct_pack::format]") {
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("!ci")) == 2);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING(">ci")) == 2);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("<ci")) == 2);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("=ci")) == 2);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("@ci")) == 2);
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("ci")) == 2);

    // Padding sanity
    REQUIRE_STATIC(struct_pack::countItems(PY_STRING("ic")) == 2);
}

TEST_CASE("getTypeOfItem without item count", "[struct_pack::format]") {
    REQUIRE_STATIC(struct_pack::getTypeOfItem<0>(PY_STRING("cih")).formatChar
                   == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<1>(PY_STRING("cih")).formatChar
                   == 'i');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<2>(PY_STRING("cih")).formatChar
                   == 'h');
}

TEST_CASE("getTypeOfItem with format specifier", "[struct_pack::format]") {
    REQUIRE_STATIC(struct_pack::getTypeOfItem<0>(PY_STRING("cih")).formatChar
                   == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<0>(PY_STRING("@cih")).formatChar
                   == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<0>(PY_STRING(">cih")).formatChar
                   == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<0>(PY_STRING("<cih")).formatChar
                   == 'c');
}

TEST_CASE("getTypeOfItem with item count", "[struct_pack::format]") {
    REQUIRE_STATIC(struct_pack::getTypeOfItem<0>(PY_STRING("L2ci")).formatChar
                   == 'L');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<1>(PY_STRING("L2ci")).formatChar
                   == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<2>(PY_STRING("L2ci")).formatChar
                   == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<3>(PY_STRING("L2ci")).formatChar
                   == 'i');

    REQUIRE_STATIC(struct_pack::getTypeOfItem<0>(PY_STRING("c3sh")).formatChar
                   == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<1>(PY_STRING("c3sh")).formatChar
                   == 's');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<2>(PY_STRING("c3sh")).formatChar
                   == 'h');

    // <L2c5si
    REQUIRE_STATIC(
        struct_pack::getTypeOfItem<0>(PY_STRING("<L2c5si")).formatChar == 'L');

    REQUIRE_STATIC(
        struct_pack::getTypeOfItem<1>(PY_STRING("<L2c5si")).formatChar == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<1>(PY_STRING("<L2c5si")).size
                   == 1);

    REQUIRE_STATIC(
        struct_pack::getTypeOfItem<2>(PY_STRING("<L2c5si")).formatChar == 'c');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<2>(PY_STRING("<L2c5si")).size
                   == 1);

    REQUIRE_STATIC(
        struct_pack::getTypeOfItem<3>(PY_STRING("<L2c5si")).formatChar == 's');
    REQUIRE_STATIC(struct_pack::getTypeOfItem<3>(PY_STRING("<L2c5si")).size
                   == 5);

    REQUIRE_STATIC(
        struct_pack::getTypeOfItem<4>(PY_STRING("<L2c5si")).formatChar == 'i');
}

TEST_CASE("getBinaryOffset with item count", "[struct_pack::format]") {
#ifdef _MSC_VER
    REQUIRE_STATIC(struct_pack::getBinaryOffset<0>(PY_STRING("L2c5si")) == 0);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<1>(PY_STRING("L2c5si")) == 4);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<2>(PY_STRING("L2c5si")) == 5);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<3>(PY_STRING("L2c5si")) == 6);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<4>(PY_STRING("L2c5si")) == 12);
#else
    REQUIRE_STATIC(struct_pack::getBinaryOffset<0>(PY_STRING("L2c5si")) == 0);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<1>(PY_STRING("L2c5si")) == 8);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<2>(PY_STRING("L2c5si")) == 9);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<3>(PY_STRING("L2c5si")) == 10);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<4>(PY_STRING("L2c5si")) == 16);
#endif

    REQUIRE_STATIC(struct_pack::getBinaryOffset<0>(PY_STRING("<L2c5si")) == 0);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<1>(PY_STRING("<L2c5si")) == 4);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<2>(PY_STRING("<L2c5si")) == 5);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<3>(PY_STRING("<L2c5si")) == 6);
    REQUIRE_STATIC(struct_pack::getBinaryOffset<4>(PY_STRING("<L2c5si")) == 11);
}
