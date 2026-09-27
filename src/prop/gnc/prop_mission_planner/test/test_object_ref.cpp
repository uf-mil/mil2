#include <gtest/gtest.h>

#include <string>

#include "prop_mission_planner/object_ref.hpp"

using prop_mission_planner::ObjectRef;
using prop_mission_planner::parse_object_ref;

TEST(ObjectRef, ParsesAPoint)
{
    auto const ref = parse_object_ref("-4.0;-4.0");
    ASSERT_TRUE(ref.has_value());
    EXPECT_DOUBLE_EQ(ref->point.x, -4.0);
    EXPECT_DOUBLE_EQ(ref->point.y, -4.0);
}

TEST(ObjectRef, ToleratesWhitespace)
{
    auto const ref = parse_object_ref(" 20 ; -3 ");
    ASSERT_TRUE(ref.has_value());
    EXPECT_DOUBLE_EQ(ref->point.x, 20.0);
    EXPECT_DOUBLE_EQ(ref->point.y, -3.0);
}

TEST(ObjectRef, RejectsAnythingElse)
{
    for (char const *bad : { "", "4", "a;b", "4;", ";4", "4;5;6", "4,5", "4;5x", "nan;0", "inf;0" })
    {
        EXPECT_FALSE(parse_object_ref(bad).has_value()) << "accepted \"" << bad << "\"";
    }
}

TEST(ObjectRef, RoundTripsThroughBehaviorTreeStrings)
{
    ObjectRef const original{ prop_maneuvers::Point{ 23.456, -0.5 } };
    auto const back = BT::convertFromString<ObjectRef>(BT::toStr(original));
    EXPECT_NEAR(back.point.x, 23.456, 1e-3);
    EXPECT_NEAR(back.point.y, -0.5, 1e-3);
}

TEST(ObjectRef, ConvertFromStringThrowsOnBadText)
{
    EXPECT_THROW((void)BT::convertFromString<ObjectRef>("4"), BT::RuntimeError);
}
