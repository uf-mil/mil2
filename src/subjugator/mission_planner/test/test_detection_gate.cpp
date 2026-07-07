#include <gtest/gtest.h>

#include <vector>

#include "detection_gate.hpp"

using detection_gate::MissGate;
using Verdict = detection_gate::MissGate::Verdict;

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
    EXPECT_EQ(g.hits, 0);
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
    EXPECT_EQ(g.hits, 0);
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

TEST(MissGate, ConsecutiveHitsTracked)
{
    // hits counts consecutive fresh frames WITH the target (positive
    // confirmation for DescendUntilDetected / DetectTarget / ConfirmGrasp).
    MissGate g;
    EXPECT_EQ(g.update(true, 100, 5), Verdict::kHit);
    EXPECT_EQ(g.hits, 1);
    // stale re-read of the hit frame does not inflate the count
    EXPECT_EQ(g.update(true, 100, 5), Verdict::kStale);
    EXPECT_EQ(g.hits, 1);
    EXPECT_EQ(g.update(true, 200, 5), Verdict::kHit);
    EXPECT_EQ(g.hits, 2);
    // a fresh miss breaks the streak
    EXPECT_EQ(g.update(false, 300, 5), Verdict::kMiss);
    EXPECT_EQ(g.hits, 0);
    EXPECT_EQ(g.update(true, 400, 5), Verdict::kHit);
    EXPECT_EQ(g.hits, 1);
}

TEST(MissGate, SeedIgnoresPreexistingFrame)
{
    // seed() from onStart: the cached array present when the node starts must
    // never count — only frames captured after start are evidence (kills
    // acting on a pre-move / pre-lift observation).
    MissGate g;
    g.seed(500);
    EXPECT_EQ(g.update(true, 500, 5), Verdict::kStale);  // the pre-start frame
    EXPECT_EQ(g.hits, 0);
    EXPECT_EQ(g.update(true, 600, 5), Verdict::kHit);  // first post-start frame
    EXPECT_EQ(g.hits, 1);
}

TEST(MissGate, SeedClearsCounters)
{
    MissGate g;
    (void)g.update(true, 100, 5);
    (void)g.update(false, 200, 5);
    g.seed(300);
    EXPECT_EQ(g.hits, 0);
    EXPECT_EQ(g.misses, 0);
    EXPECT_EQ(g.update(false, 400, 5), Verdict::kMiss);
    EXPECT_EQ(g.misses, 1);
}

TEST(MissGate, SeedCannotProtectUnstampedFrames)
{
    // Documented degradation: stamp==0 frames are ALWAYS fresh, so seeding
    // cannot fence off a cached pre-start frame from an unstamped publisher —
    // every re-read counts and thresholds degrade to per-tick. This test
    // pins the behavior so a future 'fix' is a conscious decision.
    MissGate g;
    g.seed(0);
    EXPECT_EQ(g.update(true, 0, 5), Verdict::kHit);  // same cached frame, still counted
    EXPECT_EQ(g.update(true, 0, 5), Verdict::kHit);
    EXPECT_EQ(g.hits, 2);
}

// Minimal mock of yolo_msgs::msg::DetectionArray for the pure-header helpers.
namespace
{
struct MockDetection
{
    std::string class_name;
    double score;
};
struct MockStamp
{
    std::int32_t sec;
    std::uint32_t nanosec;
};
struct MockHeader
{
    MockStamp stamp;
};
struct MockArray
{
    MockHeader header;
    std::vector<MockDetection> detections;
};
}  // namespace

TEST(DetectionGateHelpers, ContainsLabelRespectsConfFloor)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.35 }, { "nut_cylinder", 0.90 } } };
    EXPECT_TRUE(detection_gate::contains_label(arr, "nut_cylinder", 0.60));
    EXPECT_FALSE(detection_gate::contains_label(arr, "table", 0.40));  // below floor
    EXPECT_FALSE(detection_gate::contains_label(arr, "bandaid_box", 0.10));
}

TEST(DetectionGateHelpers, BestDetectionPicksHighestConfMatch)
{
    MockArray arr{ { { 3, 500 } },
                   { { "table", 0.35 }, { "nut_cylinder", 0.50 }, { "nut_cylinder", 0.90 }, { "table", 0.99 } } };
    // highest-score detection among the label matches at/above the floor
    auto const* best = detection_gate::best_detection(arr, "nut_cylinder", 0.30);
    ASSERT_NE(best, nullptr);
    EXPECT_DOUBLE_EQ(best->score, 0.90);
    EXPECT_EQ(best->class_name, "nut_cylinder");
}

TEST(DetectionGateHelpers, BestDetectionRespectsConfFloorAndMissing)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.35 }, { "nut_cylinder", 0.50 } } };
    // every match is below the floor -> no selection
    EXPECT_EQ(detection_gate::best_detection(arr, "table", 0.40), nullptr);
    // label absent entirely
    EXPECT_EQ(detection_gate::best_detection(arr, "bandaid_box", 0.10), nullptr);
    // empty array
    MockArray empty{ { { 0, 0 } }, {} };
    EXPECT_EQ(detection_gate::best_detection(empty, "table", 0.10), nullptr);
}

TEST(DetectionGateHelpers, StampNsOfAndSeedFrom)
{
    MockArray arr{ { { 3, 500 } }, {} };
    EXPECT_EQ(MissGate::stamp_ns_of(arr), 3000000500LL);

    MissGate g;
    g.seed_from(std::optional<MockArray>{ arr });
    // the seeded (pre-start) frame is stale; a newer one is fresh
    EXPECT_EQ(g.update(true, 3000000500LL, 5), Verdict::kStale);
    EXPECT_EQ(g.update(true, 3000000501LL, 5), Verdict::kHit);

    // no cached array at start -> plain reset (everything fresh)
    MissGate g2;
    g2.seed_from(std::optional<MockArray>{});
    EXPECT_EQ(g2.update(true, 100, 5), Verdict::kHit);
}
