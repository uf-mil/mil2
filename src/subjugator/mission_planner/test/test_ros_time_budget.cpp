#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "ros_time_budget.hpp"

using ros_time_budget::Budget;

namespace
{
constexpr std::int64_t kMsec = 1000000LL;  // ns per millisecond
}

TEST(RosTimeBudget, NeverExpiredBeforeArm)
{
    Budget b;
    EXPECT_FALSE(b.expired(0));
    EXPECT_FALSE(b.expired(std::numeric_limits<std::int64_t>::max()));
}

TEST(RosTimeBudget, ExpiresAtDeadlineInclusive)
{
    Budget b;
    b.arm(1000 * kMsec, 500);
    EXPECT_FALSE(b.expired(1000 * kMsec));
    EXPECT_FALSE(b.expired(1499 * kMsec));
    // >= : the instant the deadline is reached counts as expired
    EXPECT_TRUE(b.expired(1500 * kMsec));
    EXPECT_TRUE(b.expired(1501 * kMsec));
}

TEST(RosTimeBudget, ZeroBudgetExpiresImmediatelyEvenOnFrozenClock)
{
    // A paused sim clock must not turn msec=0 into "wait forever".
    Budget b;
    b.arm(42 * kMsec, 0);
    EXPECT_TRUE(b.expired(42 * kMsec));
}

TEST(RosTimeBudget, DisarmStopsExpiry)
{
    Budget b;
    b.arm(0, 100);
    ASSERT_TRUE(b.expired(200 * kMsec));
    b.disarm();
    EXPECT_FALSE(b.expired(200 * kMsec));
}

TEST(RosTimeBudget, RearmReplacesDeadline)
{
    Budget b;
    b.arm(0, 100);
    ASSERT_TRUE(b.expired(150 * kMsec));
    b.arm(150 * kMsec, 100);
    EXPECT_FALSE(b.expired(200 * kMsec));
    EXPECT_TRUE(b.expired(250 * kMsec));
}

TEST(RosTimeBudget, ClockJumpBackwardNeverExpiresEarly)
{
    // use_sim_time=false + an NTP step back must not fire the timeout;
    // the budget just runs correspondingly longer.
    Budget b;
    b.arm(1000 * kMsec, 100);
    EXPECT_FALSE(b.expired(500 * kMsec));
    EXPECT_TRUE(b.expired(1100 * kMsec));
}

TEST(RosTimeBudget, LargeBudgetDoesNotOverflow)
{
    // 45 s (relative_move's biggest caller) and far beyond stay in int64 ns.
    Budget b;
    b.arm(0, std::numeric_limits<int>::max());
    EXPECT_FALSE(b.expired(0));
    EXPECT_TRUE(b.expired(static_cast<std::int64_t>(std::numeric_limits<int>::max()) * kMsec));
}
