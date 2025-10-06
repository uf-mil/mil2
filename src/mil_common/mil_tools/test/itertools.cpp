#include "mil_tools/itertools.hpp"

#include <gtest/gtest.h>

#include <cstdlib>

#include "mil_tools/noncopyable.hpp"

class nocopy : public mil::noncopyable
{
    int x_;

  public:
    explicit nocopy(int x) : x_(x)
    {
    }
    bool operator==(nocopy const& other) const
    {
        return x_ == other.x_;
    }
};

TEST(mil_tools_itertools, enumerate)
{
    std::vector<std::pair<int, int>> exp_nums = { std::make_pair(0, 10), std::make_pair(1, 20), std::make_pair(2, 30) };
    std::vector<int> sample_nums = { 10, 20, 30 };
    std::vector<std::pair<int, int>> test_nums;
    for (auto const& [index, value] : mil::itertools::enumerate(sample_nums))
    {
        test_nums.emplace_back(std::make_pair(index, value));
    }
    EXPECT_EQ(test_nums, exp_nums);

    std::vector<std::pair<int, std::string>> exp_strings = { std::make_pair(0, "first"), std::make_pair(1, "second"),
                                                             std::make_pair(2, "third") };
    std::vector<std::string> sample_strings = { "first", "second", "third" };
    std::vector<std::pair<int, std::string>> test_strings;
    for (auto const& [index, value] : mil::itertools::enumerate(sample_strings))
    {
        test_strings.emplace_back(std::make_pair(index, value));
    }
    EXPECT_EQ(test_strings, exp_strings);

    // Ensure that enumerate() does not produce copies
    std::vector<nocopy> sample_nocopy;
    for (int i = 37; i < 47; i++)
    {
        sample_nocopy.emplace_back(i);
    }
    int nocopy_count = 0;
    for (auto const& [index, value] : mil::itertools::enumerate(sample_nocopy))
    {
        EXPECT_EQ(index, nocopy_count);
        EXPECT_EQ(&value, &sample_nocopy[nocopy_count]);
        nocopy_count++;
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
