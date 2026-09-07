#include <gtest/gtest.h>

#include "select_target_logic.hpp"

using namespace select_target;

TEST(ParseLabels, SplitsAndTrims)
{
    auto v = parse_labels("nut_bolt, plug ,  ");
    ASSERT_EQ(v.size(), 2u);
    EXPECT_EQ(v[0], "nut_bolt");
    EXPECT_EQ(v[1], "plug");
}

TEST(LabelsForRole, MapsRoles)
{
    EXPECT_EQ(labels_for_role("survey_repair", "nut_bolt,plug", "pill,bandage").size(), 2u);
    EXPECT_EQ(labels_for_role("search_rescue", "nut_bolt,plug", "pill,bandage")[0], "pill");
    EXPECT_TRUE(labels_for_role("", "nut_bolt,plug", "pill,bandage").empty());
    EXPECT_TRUE(labels_for_role("bogus", "nut_bolt,plug", "pill,bandage").empty());
}

TEST(PickBest, HighestScoreInTargets)
{
    std::vector<Candidate> c{ { "plug", 0.7 }, { "nut_bolt", 0.9 }, { "fish", 0.99 } };
    auto best = pick_best(c, { "nut_bolt", "plug" }, 0.60, "");
    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(*best, "nut_bolt");
}

TEST(PickBest, RespectsMinConf)
{
    std::vector<Candidate> c{ { "plug", 0.55 } };
    EXPECT_FALSE(pick_best(c, { "nut_bolt", "plug" }, 0.60, "").has_value());
}

TEST(PickBest, RespectsExclude)
{
    std::vector<Candidate> c{ { "nut_bolt", 0.9 }, { "plug", 0.8 } };
    auto best = pick_best(c, { "nut_bolt", "plug" }, 0.60, "nut_bolt");
    ASSERT_TRUE(best.has_value());
    EXPECT_EQ(*best, "plug");
}

TEST(PickBest, NoneQualify)
{
    std::vector<Candidate> c{ { "fish", 0.99 } };
    EXPECT_FALSE(pick_best(c, { "nut_bolt", "plug" }, 0.60, "").has_value());
}
