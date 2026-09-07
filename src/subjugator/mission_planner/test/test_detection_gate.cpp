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
struct MockVec2
{
    double x;
    double y;
};
struct MockBBox
{
    MockVec2 size;
};
struct MockDetection
{
    std::string class_name;
    double score;
    // Defaulted so the score-only cases below stay readable; only the
    // area-selection tests care about the box.
    MockBBox bbox{};
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

// The octagon down-cam model emits a tiny, VERY confident phantom 'table' box in
// a frame corner alongside the real table, which it scores far lower. Picking by
// score alone therefore locks onto the phantom, so the sub chases a point ~0.44 m
// from the real table even when it is already sitting over it. Measured on this
// model: real table ~0.60x0.40 of frame at conf 0.24-0.42, phantom ~0.03x0.06 at
// conf 0.85. Area separates them by an order of magnitude; confidence inverts.
TEST(DetectionGateHelpers, LargestAreaSelectsRealTableOverMoreConfidentPhantom)
{
    MockArray arr{ { { 3, 500 } },
                   { { "table", 0.85, { { 30.0, 36.0 } } },        // phantom: high score, tiny box
                     { "table", 0.31, { { 576.0, 240.0 } } } } };  // real table: low score, big box

    auto const* best = detection_gate::best_detection(arr, "table", 0.20, detection_gate::Select::kLargestArea);
    ASSERT_NE(best, nullptr);
    EXPECT_DOUBLE_EQ(best->score, 0.31);
    EXPECT_DOUBLE_EQ(best->bbox.size.x, 576.0);
}

// The area rule must stay opt-in: the small grasp props are selected with the
// same helper, and there "biggest box wins" would be actively wrong.
TEST(DetectionGateHelpers, DefaultSelectionStillPicksHighestConfidence)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.85, { { 30.0, 36.0 } } }, { "table", 0.31, { { 576.0, 240.0 } } } } };

    auto const* best = detection_gate::best_detection(arr, "table", 0.20);
    ASSERT_NE(best, nullptr);
    EXPECT_DOUBLE_EQ(best->score, 0.85);
}

// The confidence floor and "no match" contract must not change with the mode.
TEST(DetectionGateHelpers, LargestAreaStillRespectsConfFloorAndMissing)
{
    MockArray arr{ { { 3, 500 } },
                   { { "table", 0.15, { { 576.0, 240.0 } } },  // biggest, but under the floor
                     { "table", 0.55, { { 30.0, 36.0 } } } } };

    auto const* best = detection_gate::best_detection(arr, "table", 0.20, detection_gate::Select::kLargestArea);
    ASSERT_NE(best, nullptr);
    EXPECT_DOUBLE_EQ(best->score, 0.55);

    EXPECT_EQ(detection_gate::best_detection(arr, "bandaid_box", 0.10, detection_gate::Select::kLargestArea), nullptr);
}

// The mode arrives as a BT port string. An unknown or empty value must fall back
// to the confidence default rather than silently enabling the area rule on props.
TEST(DetectionGateHelpers, SelectFromParsesKnownModesAndDefaultsSafely)
{
    EXPECT_EQ(detection_gate::select_from("largest"), detection_gate::Select::kLargestArea);
    EXPECT_EQ(detection_gate::select_from("confidence"), detection_gate::Select::kConfidence);
    EXPECT_EQ(detection_gate::select_from(""), detection_gate::Select::kConfidence);
    EXPECT_EQ(detection_gate::select_from("biggest"), detection_gate::Select::kConfidence);
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

// --- SizeGate: reject boxes too small to plausibly be the target -------------
//
// Frame-fraction reference for a 960x600 down cam (W*H = 576000 px):
//   30 x  36 =   1080 px = 0.0019 of frame  -- phantom, from the 621e0955 sample
//  220 x 212 =  46640 px = 0.081  of frame  -- LARGEST measured artifact
//  190 x 182 =  34580 px = 0.060  of frame  -- grasp prop at hone altitude
//  360 x 240 =  86400 px = 0.150  of frame  -- exactly the table floor
//  576 x 240 = 138240 px = 0.240  of frame  -- real table, measured
constexpr std::uint32_t kW = 960;
constexpr std::uint32_t kH = 600;

// THE BUG: with the real table out of frame the model still emits an artifact
// above the 0.40 confidence floor, so DescendUntilDetected / DetectTarget /
// SearchForTarget all read "table present" and S2 never spiral-searches.
TEST(SizeGateTest, AreaFloorRejectsArtifactOnlyPresence)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.79, { { 220.0, 212.0 } } } } };
    detection_gate::SizeGate const floor{ 0.15, kW, kH };
    EXPECT_FALSE(detection_gate::contains_label(arr, "table", 0.40, floor));
}

// The filter is opt-in: an absent SizeGate must preserve today's behaviour
// exactly, including for start_gate_mission.xml's shark.
TEST(SizeGateTest, DefaultGateAcceptsEverything)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.79, { { 220.0, 212.0 } } } } };
    EXPECT_TRUE(detection_gate::contains_label(arr, "table", 0.40));
}

// REGRESSION GUARD. The grasp props subtend 0.04-0.11 of frame at hone
// altitude -- the same size as the artifacts. The table's floor applied to a
// prop would reject it in nearly every frame and make S4 dead-reckon every
// grasp, so the props must keep the default gate.
TEST(SizeGateTest, DefaultGateStillSeesPropSizedBoxes)
{
    MockArray arr{ { { 3, 500 } }, { { "nut_cylinder", 0.62, { { 190.0, 182.0 } } } } };
    EXPECT_TRUE(detection_gate::contains_label(arr, "nut_cylinder", 0.40));
}

TEST(SizeGateTest, AreaFloorAcceptsRealTable)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.31, { { 576.0, 240.0 } } } } };
    detection_gate::SizeGate const floor{ 0.15, kW, kH };
    EXPECT_TRUE(detection_gate::contains_label(arr, "table", 0.20, floor));
}

TEST(SizeGateTest, AreaFloorIsInclusive)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.50, { { 360.0, 240.0 } } } } };  // exactly 0.15
    detection_gate::SizeGate const floor{ 0.15, kW, kH };
    EXPECT_TRUE(detection_gate::contains_label(arr, "table", 0.20, floor));
}

// An unverifiable frame must never be reported as a confirmed sighting. The
// nodes hold RUNNING before reaching this, but the helper must not guess.
TEST(SizeGateTest, FailsClosedWithoutImageSize)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.31, { { 576.0, 240.0 } } } } };
    detection_gate::SizeGate const no_size{ 0.15, 0, 0 };
    EXPECT_FALSE(detection_gate::contains_label(arr, "table", 0.20, no_size));
    EXPECT_EQ(detection_gate::best_detection(arr, "table", 0.20, detection_gate::Select::kLargestArea, no_size),
              nullptr);
}

// Presence and selection must qualify candidates through the SAME predicate:
// largest-area alone still returns the phantom when the phantom is all there is.
TEST(SizeGateTest, BestDetectionRejectsArtifactOnlyFrame)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.79, { { 220.0, 212.0 } } } } };
    detection_gate::SizeGate const floor{ 0.15, kW, kH };
    EXPECT_EQ(detection_gate::best_detection(arr, "table", 0.40, detection_gate::Select::kLargestArea, floor), nullptr);
}

TEST(SizeGateTest, BestDetectionKeepsRealTableWhenBothPresent)
{
    MockArray arr{ { { 3, 500 } },
                   { { "table", 0.85, { { 30.0, 36.0 } } },        // phantom
                     { "table", 0.31, { { 576.0, 240.0 } } } } };  // real table
    detection_gate::SizeGate const floor{ 0.15, kW, kH };
    auto const* best = detection_gate::best_detection(arr, "table", 0.20, detection_gate::Select::kLargestArea, floor);
    ASSERT_NE(best, nullptr);
    EXPECT_DOUBLE_EQ(best->bbox.size.x, 576.0);

    // The floor alone is enough to fix the RANKING: with the phantom filtered
    // out, even confidence-mode selection returns the real table.
    auto const* by_conf =
        detection_gate::best_detection(arr, "table", 0.20, detection_gate::Select::kConfidence, floor);
    ASSERT_NE(by_conf, nullptr);
    EXPECT_DOUBLE_EQ(by_conf->bbox.size.x, 576.0);
}

// A negative floor must DISABLE the filter, not invert it. Pinned because the
// value arrives from an XML port where any double parses: a typo that lands
// negative silently restores the phantom bug this filter exists to fix.
TEST(SizeGateTest, NegativeFloorDisablesTheFilter)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.79, { { 220.0, 212.0 } } } } };
    detection_gate::SizeGate const negative{ -1.0, kW, kH };
    EXPECT_TRUE(detection_gate::contains_label(arr, "table", 0.40, negative));
}

// A floor above 1.0 is unsatisfiable and rejects everything -- the likely typo
// being "15" for 15% or "1.5" for 0.15. Pinned so the symptom is a documented
// contract rather than a mystery perception failure.
TEST(SizeGateTest, FloorAboveOneRejectsEverything)
{
    MockArray arr{ { { 3, 500 } }, { { "table", 0.97, { { 900.0, 560.0 } } } } };  // 0.875 of frame
    detection_gate::SizeGate const impossible{ 15.0, kW, kH };
    EXPECT_FALSE(detection_gate::contains_label(arr, "table", 0.40, impossible));
}
