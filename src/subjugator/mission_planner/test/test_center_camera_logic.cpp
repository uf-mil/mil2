#include <gtest/gtest.h>

#include "center_camera_logic.hpp"

using center_camera::MissGate;
using Verdict = center_camera::MissGate::Verdict;

TEST(MissGate, HitThenFlickerThenHitNeverLost)
{
    MissGate g;
    EXPECT_EQ(g.update(true, 100, 5), Verdict::kHit);
    // 4 fresh misses with miss_frames=5: tolerated as transient flicker
    EXPECT_EQ(g.update(false, 200, 5), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 300, 5), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 400, 5), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 500, 5), Verdict::kMiss);
    // hit resets the miss counter
    EXPECT_EQ(g.update(true, 600, 5), Verdict::kHit);
    EXPECT_EQ(g.update(false, 700, 5), Verdict::kMiss);
    EXPECT_EQ(g.misses, 1);
}

TEST(MissGate, LostAfterNConsecutiveFreshMisses)
{
    MissGate g;
    EXPECT_EQ(g.update(true, 100, 5), Verdict::kHit);
    EXPECT_EQ(g.update(false, 200, 5), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 300, 5), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 400, 5), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 500, 5), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 600, 5), Verdict::kLost);
}

TEST(MissGate, StaleFramesNeverCount)
{
    MissGate g;
    EXPECT_EQ(g.update(false, 100, 5), Verdict::kMiss);
    // same frame re-read on subsequent BT ticks: no double counting
    EXPECT_EQ(g.update(false, 100, 5), Verdict::kStale);
    EXPECT_EQ(g.update(false, 100, 5), Verdict::kStale);
    EXPECT_EQ(g.misses, 1);
    // stale hit is also stale (no reset from a re-read)
    EXPECT_EQ(g.update(true, 100, 5), Verdict::kStale);
    EXPECT_EQ(g.misses, 1);
}

TEST(MissGate, MissFramesOneIsLegacyInstantFail)
{
    MissGate g;
    EXPECT_EQ(g.update(false, 100, 1), Verdict::kLost);
}

TEST(MissGate, ZeroStampAlwaysFresh)
{
    // Publisher that doesn't stamp: every read counts (degenerate but never stalls).
    MissGate g;
    EXPECT_EQ(g.update(false, 0, 3), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 0, 3), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 0, 3), Verdict::kLost);
    EXPECT_EQ(g.update(true, 0, 3), Verdict::kHit);
    EXPECT_EQ(g.misses, 0);
}

TEST(MissGate, ResetClearsState)
{
    MissGate g;
    (void)g.update(false, 100, 5);
    (void)g.update(false, 200, 5);
    g.reset();
    EXPECT_EQ(g.misses, 0);
    // old stamps are fresh again after reset
    EXPECT_EQ(g.update(false, 100, 5), Verdict::kMiss);
    EXPECT_EQ(g.misses, 1);
}

TEST(MissGate, LostKeepsReportingLostWhileMissing)
{
    // After the threshold, further fresh misses still report kLost (the caller
    // returns FAILURE and halts, but a re-tick must not flip back to kMiss).
    MissGate g;
    EXPECT_EQ(g.update(false, 100, 2), Verdict::kMiss);
    EXPECT_EQ(g.update(false, 200, 2), Verdict::kLost);
    EXPECT_EQ(g.update(false, 300, 2), Verdict::kLost);
}
