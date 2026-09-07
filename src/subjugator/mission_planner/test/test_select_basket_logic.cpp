#include <gtest/gtest.h>

#include "select_basket_logic.hpp"

using namespace select_basket;

TEST(MarkerForRole, MapsKnownRoles)
{
    EXPECT_EQ(marker_for_role("survey_repair", "warning", "red_cross"), "warning");
    EXPECT_EQ(marker_for_role("search_rescue", "warning", "red_cross"), "red_cross");
}

TEST(MarkerForRole, HonorsCustomMarkers)
{
    EXPECT_EQ(marker_for_role("survey_repair", "hazard_icon", "medical_icon"), "hazard_icon");
    EXPECT_EQ(marker_for_role("search_rescue", "hazard_icon", "medical_icon"), "medical_icon");
}

TEST(MarkerForRole, UnknownOrEmptyRoleIsEmpty)
{
    EXPECT_TRUE(marker_for_role("", "warning", "red_cross").empty());
    EXPECT_TRUE(marker_for_role("bogus", "warning", "red_cross").empty());
}
