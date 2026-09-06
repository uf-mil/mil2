#include <gtest/gtest.h>

#include <limits>

#include "descend_floor.hpp"

using descend::next_descend_z;

namespace
{
constexpr double kStep = 0.20;
constexpr double kTol = 0.10;
// The Task 5 octagon table: tabletop at world z = -0.85 (props rest at -0.80
// with 0.12-0.20 m heights), so a floor of -0.55 keeps the sub 0.30 m above the
// tabletop and ~0.15 m above the tallest prop.
constexpr double kFloor = -0.55;
}  // namespace

// Default (no floor) must behave exactly as the node did before: every step is
// a full step_m, forever. A regression here silently re-bounds an unbounded
// descent, or vice versa.
TEST(DescendFloor, NoFloorAlwaysStepsAFullStep)
{
    double const none = -std::numeric_limits<double>::infinity();
    auto z = next_descend_z(-0.35, kStep, none, kTol);
    ASSERT_TRUE(z.has_value());
    EXPECT_DOUBLE_EQ(*z, -0.55);

    z = next_descend_z(-100.0, kStep, none, kTol);
    ASSERT_TRUE(z.has_value());
    EXPECT_DOUBLE_EQ(*z, -100.2);
}

TEST(DescendFloor, StepsNormallyWhileClearOfTheFloor)
{
    auto const z = next_descend_z(-0.15, kStep, kFloor, kTol);
    ASSERT_TRUE(z.has_value());
    EXPECT_DOUBLE_EQ(*z, -0.35);
}

// The last step is clipped to the floor rather than overshooting it.
TEST(DescendFloor, ClampsTheFinalStepToTheFloor)
{
    auto const z = next_descend_z(-0.35, kStep, kFloor, kTol);
    ASSERT_TRUE(z.has_value());
    EXPECT_DOUBLE_EQ(*z, kFloor);
}

// At the floor there is nothing left to command: no value == "give up".
TEST(DescendFloor, RefusesWhenAlreadyAtTheFloor)
{
    EXPECT_FALSE(next_descend_z(kFloor, kStep, kFloor, kTol).has_value());
}

TEST(DescendFloor, RefusesWhenAlreadyBelowTheFloor)
{
    EXPECT_FALSE(next_descend_z(-1.41, kStep, kFloor, kTol).has_value());
}

// A remaining descent shorter than pos_tol is indistinguishable from having
// arrived -- the node's own goal-reached check uses pos_tol -- so commanding it
// would spin: the goal would read as reached the instant it was issued.
TEST(DescendFloor, RefusesAStepShorterThanTheGoalTolerance)
{
    EXPECT_FALSE(next_descend_z(-0.50, kStep, kFloor, kTol).has_value());
    EXPECT_FALSE(next_descend_z(-0.46, kStep, kFloor, kTol).has_value());
}

// Exactly pos_tol of room left is still a usable step (boundary pinned so a
// later >= / > edit is caught).
TEST(DescendFloor, AcceptsAStepExactlyTheGoalTolerance)
{
    auto const z = next_descend_z(-0.45, kStep, kFloor, kTol);
    ASSERT_TRUE(z.has_value());
    EXPECT_DOUBLE_EQ(*z, kFloor);
}

// The regression this whole floor exists to prevent: from the `near` start at
// z=-0.35, the old code stepped to -1.41, which is 0.56 m BELOW the tabletop --
// a down camera there cannot see the table at any lateral offset.
TEST(DescendFloor, NeverDescendsBelowTheTabletop)
{
    double z = -0.35;
    for (int i = 0; i < 12; ++i)
    {
        auto const next = next_descend_z(z, kStep, kFloor, kTol);
        if (!next.has_value())
        {
            break;
        }
        z = *next;
    }
    EXPECT_GE(z, kFloor);
    EXPECT_GT(z, -0.85);
}
