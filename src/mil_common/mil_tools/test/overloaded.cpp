#include "mil_tools/overloaded.hpp"

#include <gtest/gtest.h>

TEST(Overloaded, Test)
{
    auto f = mil::overloaded::make{
        [](int const& x) { return x + 1; },
        [](double const& x) { return x + 2; },
        [](auto const& x) { return x + "3"; },
    };

    EXPECT_EQ(f(1), 2);
    EXPECT_EQ(f(1.0), 3.0);
    EXPECT_EQ(f(std::string("1")), "13");
}
