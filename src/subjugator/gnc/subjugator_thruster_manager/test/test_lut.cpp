#include <gtest/gtest.h>

#include <cmath>

#include "subjugator_thruster_manager/lut.h"

// The endpoints of the measured T200 curve: full reverse is weaker than full
// forward, which is the asymmetry the whole change exists to capture.
constexpr double kFullReverse = -39.9130655;
constexpr double kFullForward = 51.4849125;

TEST(Lut, EndpointsMatchTable)
{
    EXPECT_DOUBLE_EQ(force_from_effort(-1.0), kFullReverse);
    EXPECT_DOUBLE_EQ(force_from_effort(1.0), kFullForward);
}

TEST(Lut, ClampsOutOfRangeEffort)
{
    EXPECT_DOUBLE_EQ(force_from_effort(-2.0), kFullReverse);
    EXPECT_DOUBLE_EQ(force_from_effort(5.0), kFullForward);
}

TEST(Lut, ClampsOutOfRangeForce)
{
    EXPECT_DOUBLE_EQ(effort_from_force(kFullReverse - 100.0), -1.0);
    EXPECT_DOUBLE_EQ(effort_from_force(kFullForward + 100.0), 1.0);
}

TEST(Lut, ZeroMapsToZeroBothWays)
{
    EXPECT_DOUBLE_EQ(force_from_effort(0.0), 0.0);
    EXPECT_DOUBLE_EQ(effort_from_force(0.0), 0.0);
}

// force_from_effort must be non-decreasing across the whole effort range: more
// effort never yields less thrust.
TEST(Lut, ForceIsMonotonicInEffort)
{
    double previous = force_from_effort(-1.0);
    for (int i = -100; i <= 100; ++i)
    {
        double const effort = static_cast<double>(i) / 100.0;
        double const force = force_from_effort(effort);
        EXPECT_GE(force, previous - 1e-12) << "non-monotonic at effort " << effort;
        previous = force;
    }
}

// effort_from_force inverts force_from_effort. We skip the central deadband
// (|effort| <= 0.07) where many efforts collapse to 0 N and the inverse is not
// unique by construction.
TEST(Lut, RoundTripEffortForceEffort)
{
    for (int i = -100; i <= 100; ++i)
    {
        double const effort = static_cast<double>(i) / 100.0;
        if (std::abs(effort) <= 0.07)
        {
            continue;
        }
        double const force = force_from_effort(effort);
        double const recovered = effort_from_force(force);
        EXPECT_NEAR(recovered, effort, 1e-9) << "round trip failed at effort " << effort;
    }
}

// The core asymmetry: forward thrust is stronger than reverse at equal effort
// magnitude, so the same effort magnitude produces a larger force forward.
TEST(Lut, ForwardStrongerThanReverse)
{
    for (double effort = 0.1; effort < 1.0; effort += 0.1)
    {
        double const forward = force_from_effort(effort);
        double const reverse = force_from_effort(-effort);
        EXPECT_GT(forward, std::abs(reverse)) << "forward not stronger at effort " << effort;
    }
}

// Equivalently, reaching a given force magnitude in reverse costs more effort
// than reaching it forward. Pick a force achievable in both directions.
TEST(Lut, ReverseCostsMoreEffortForSameForce)
{
    double const force = 30.0;
    double const forward_effort = effort_from_force(force);
    double const reverse_effort = effort_from_force(-force);
    EXPECT_GT(std::abs(reverse_effort), forward_effort);
}

// This mirrors the real vehicle/sim data flow: the thruster manager turns a
// desired per-thruster force into a normalized effort (effort_from_force) and
// publishes /thruster_efforts; the Gazebo ThrusterBridge turns that effort back
// into force (force_from_effort) and applies it to the plant. The force the sub
// actually feels must equal the force the manager commanded. Because both ends
// linearly interpolate the same table, this round trip is exact to fp precision
// for any force in the achievable range.
TEST(Lut, CommandedForceSurvivesEffortRoundTrip)
{
    for (double force = kFullReverse; force <= kFullForward; force += 0.123)
    {
        double const effort = effort_from_force(force);    // thruster manager
        double const applied = force_from_effort(effort);  // sim ThrusterBridge
        EXPECT_NEAR(applied, force, 1e-9) << "plant force diverged at commanded " << force << " N";
    }
    // Endpoints exactly.
    EXPECT_DOUBLE_EQ(force_from_effort(effort_from_force(kFullReverse)), kFullReverse);
    EXPECT_DOUBLE_EQ(force_from_effort(effort_from_force(kFullForward)), kFullForward);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
