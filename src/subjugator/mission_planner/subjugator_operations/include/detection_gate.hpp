#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>

// Pure frame-gating logic shared by every BT node that renders a verdict from
// the YOLO detection stream (CenterCamera, DescendUntilDetected, DetectTarget,
// ConfirmGraspByScale). Unit-tested without ROS/BT, following the
// select_target_logic.hpp pattern.
namespace detection_gate
{

// Distinguishes "target genuinely lost/absent" from a transient detection
// flicker, and "target genuinely present" from a single spurious frame, by
// counting consecutive *fresh* detection frames. One glinted, occluded, or
// misclassified frame must never decide a mission-level outcome by itself.
//
// Freshness is judged by the detection array's stamp: a frame is fresh when
// its stamp is newer than the last frame considered, so re-reads of the same
// array on fast BT ticks never advance either counter — thresholds mean
// "N distinct camera observations agree", independent of tick rate.
// stamp==0 (publisher not stamping) is always treated as fresh so the gate
// never stalls; the counts then degrade to per-tick (documented legacy case).
//
// Counters:
//   misses — consecutive fresh frames WITHOUT the target (reset by a hit).
//            Crossing miss_frames yields kLost ("it's really gone").
//   hits   — consecutive fresh frames WITH the target (reset by a miss).
//            Callers needing positive confirmation (e.g. "stop descending",
//            "guard passed") check `hits >= N` after a kHit verdict.
struct MissGate
{
    std::int64_t last_seen_ns{ -1 };  // stamp of the last frame considered
    int misses{ 0 };                  // consecutive fresh frames without the target
    int hits{ 0 };                    // consecutive fresh frames with the target

    enum class Verdict
    {
        kStale,  // same frame as last time -> caller should hold (RUNNING)
        kHit,    // fresh frame, target present -> caller may act / count hits
        kMiss,   // fresh frame, target absent, under threshold -> hold (RUNNING);
                 // callers confirming anything "consecutive" across frames
                 // (e.g. a settle counter) must reset that count here
        kLost    // >= miss_frames consecutive fresh misses -> caller fails
    };

    Verdict update(bool found, std::int64_t stamp_ns, int miss_frames)
    {
        if (stamp_ns != 0 && stamp_ns <= last_seen_ns)
        {
            return Verdict::kStale;
        }
        last_seen_ns = stamp_ns;
        if (found)
        {
            ++hits;
            misses = 0;
            return Verdict::kHit;
        }
        ++misses;
        hits = 0;
        return misses >= miss_frames ? Verdict::kLost : Verdict::kMiss;
    }

    // Call from onStart with the stamp of whatever detection array is already
    // cached, so only frames captured AFTER the node started can count. Kills
    // the stale-frame bugs where a node acts on a pre-move / pre-lift /
    // pre-mission observation (e.g. ConfirmGraspByScale reading a pre-lift
    // frame where the missed object still looks full-size). No effect for
    // unstamped (stamp==0) publishers — those are always "fresh".
    void seed(std::int64_t stamp_ns)
    {
        last_seen_ns = stamp_ns;
        misses = 0;
        hits = 0;
    }

    // seed() from an optional detection array (the ctx_->detections_for()
    // return), so every node's onStart is the same one-liner instead of four
    // copy-pasted blocks. Templated to keep this header pure (unit-testable
    // without ROS message types).
    template <class Arr>
    void seed_from(std::optional<Arr> const& arr)
    {
        reset();
        if (arr)
        {
            seed(stamp_ns_of(*arr));
        }
    }

    void reset()
    {
        last_seen_ns = -1;
        misses = 0;
        hits = 0;
    }

    template <class Arr>
    static std::int64_t stamp_ns_of(Arr const& arr)
    {
        return static_cast<std::int64_t>(arr.header.stamp.sec) * 1000000000LL +
               static_cast<std::int64_t>(arr.header.stamp.nanosec);
    }
};

// Optional plausibility filter layered on top of label + confidence: reject
// candidates whose box is too small to plausibly BE the target.
//
// See Select below for the phantom story; what area adds HERE is a floor, not
// a ranking. Sized against the LATER 3-pose sample (artifacts 0.018-0.081 of
// frame at conf 0.21-0.79), not the two-box sample Select quotes: the 960x600 /
// 110-deg-hfov down cam covers 5.098*h^2 m^2 of ground at camera height h, so
// the 0.662x1.183 m table fills 0.154/h^2 of the frame; and the camera cannot
// get further than h=0.63 m above the tabletop without leaving the water, so
// the real table is never below ~0.39 of frame -- ~5x that 0.081 worst case.
//
// DELIBERATELY OPT-IN (0 = disabled), for the same reason as Select: the grasp
// props subtend only 0.04-0.11 of frame at hone altitude, INSIDE that same
// 0.018-0.081 artifact band, so a table's floor applied to a prop would reject
// it in nearly every frame. There is no equivalent of select_from()'s typo guard here (any double
// parses). Call sites rendering a TABLE verdict must pass this explicitly; the
// prop sites pass 0.0 rather than omitting it, to document the intent at the
// point where someone would otherwise copy the table's value.
struct SizeGate
{
    // 0 = disabled. Deliberately unvalidated, both directions pinned by test:
    //   <= 0 (and NaN, and -inf) disables the filter -- everything
    //        passes. A typo that lands negative silently restores the phantom
    //        bug this filter exists to fix, so treat "the gate has no effect"
    //        as a first thing to check when a table read looks unfiltered.
    //   > 1.0 is unsatisfiable -- no box can ever pass, so the table becomes
    //        permanently invisible with a symptom indistinguishable from a
    //        perception failure. Likely typo: "15" for 15%, or "1.5" for 0.15.
    double min_area_frac{ 0.0 };
    std::uint32_t image_w{ 0 };
    std::uint32_t image_h{ 0 };

    bool enabled() const
    {
        return min_area_frac > 0.0;
    }

    // Fails CLOSED when the rule is on but the image size is unknown: a frame
    // we cannot verify must never be reported as a confirmed sighting. This is
    // NOT merely a backstop -- the callers deliberately differ. CenterCamera,
    // LockTargetXY and DescendUntilDetected hold RUNNING before reaching it;
    // DetectTarget lets its timeout convert it to FAILURE; SearchForTarget
    // relies on it DIRECTLY (unverifiable == "not seen" == keep spiralling).
    // Do not relax it to fail-open.
    template <class Det>
    bool passes(Det const& d) const
    {
        if (!enabled())
        {
            return true;
        }
        if (image_w == 0 || image_h == 0)
        {
            return false;
        }
        // Clamp before multiplying so a both-negative box (signs cancelling
        // into a spuriously positive area) can't pass; consistent with the
        // other detection consumers in this package (poles_big_enough.cpp,
        // hone_midpoint.cpp, determine_channel_side.cpp,
        // nav_channel_control.cpp, track_best_pair.cpp) which all clamp before
        // computing area.
        double const w = std::max(0.0, static_cast<double>(d.bbox.size.x));
        double const h = std::max(0.0, static_cast<double>(d.bbox.size.y));
        double const area = w * h;
        return area / (static_cast<double>(image_w) * static_cast<double>(image_h)) >= min_area_frac;
    }
};

// Shared presence predicate: any detection in the array with the requested
// class at or above the confidence floor. Every gate consumer must judge
// presence and stamp from the SAME array snapshot — fetch once, then call
// this and MissGate::stamp_ns_of on that one object (fetching twice can pair
// frame A's presence with frame B's stamp).
template <class Arr>
bool contains_label(Arr const& arr, std::string const& label, double min_conf, SizeGate const& size = {})
{
    for (auto const& d : arr.detections)
    {
        if (d.class_name == label && d.score >= min_conf && size.passes(d))
        {
            return true;
        }
    }
    return false;
}

// Which of several same-label candidates best_detection should return.
//
// kConfidence is the right default: for the grasp props, a bigger box usually
// just means "closer", and the most confident match is the one to trust.
//
// kLargestArea exists because the octagon down-cam model emits a tiny, highly
// confident phantom 'table' box in a frame corner in essentially every frame,
// while scoring the real table far lower (measured: phantom ~0.03x0.06 of frame
// at conf 0.85, real table ~0.60x0.40 at conf 0.24-0.42). Score alone therefore
// picks the phantom, and the consumer then steers toward a point ~0.44 m from
// the real table even when it is already over it. Area separates the two by an
// order of magnitude, and the table is by far the largest thing in a down-cam
// frame, so "biggest box wins" is a safe rule FOR THE TABLE. It is deliberately
// opt-in per call rather than global, because applying it to the small props
// would be actively wrong. The confidence floor still applies in both modes, so
// this widens nothing: it only re-ranks candidates that already qualified.
enum class Select
{
    kConfidence,
    kLargestArea,
};

// Parse the BT port spelling of Select. Anything unrecognised (including an
// empty port) falls back to kConfidence, so a typo at a call site can never
// silently turn the table rule on for the small props.
inline Select select_from(std::string const& mode)
{
    return mode == "largest" ? Select::kLargestArea : Select::kConfidence;
}

// Argmax sibling of contains_label: pointer to the best detection whose
// class_name == label and score >= min_conf, or nullptr if none match. "Best"
// means highest `score`, or largest bbox area under Select::kLargestArea.
// The nodes that need the actual detection (its bbox, center, size) all
// hand-rolled this same loop; fetch the array once and call this on that one
// snapshot (see contains_label's note on pairing presence with the stamp).
template <class Arr>
auto const* best_detection(Arr const& arr, std::string const& label, double min_conf,
                           Select select = Select::kConfidence, SizeGate const& size = {})
{
    using Det = typename std::decay_t<decltype(arr.detections)>::value_type;
    Det const* best = nullptr;
    double best_rank = -1.0;
    for (auto const& d : arr.detections)
    {
        if (d.class_name != label || d.score < min_conf || !size.passes(d))
        {
            continue;
        }
        double const rank = select == Select::kLargestArea ?
                                static_cast<double>(d.bbox.size.x) * static_cast<double>(d.bbox.size.y) :
                                static_cast<double>(d.score);
        if (rank > best_rank)
        {
            best = &d;
            best_rank = rank;
        }
    }
    return best;
}

}  // namespace detection_gate
