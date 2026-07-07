#pragma once

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

// Shared presence predicate: any detection in the array with the requested
// class at or above the confidence floor. Every gate consumer must judge
// presence and stamp from the SAME array snapshot — fetch once, then call
// this and MissGate::stamp_ns_of on that one object (fetching twice can pair
// frame A's presence with frame B's stamp).
template <class Arr>
bool contains_label(Arr const& arr, std::string const& label, double min_conf)
{
    for (auto const& d : arr.detections)
    {
        if (d.class_name == label && d.score >= min_conf)
        {
            return true;
        }
    }
    return false;
}

// Argmax sibling of contains_label: pointer to the highest-`score` detection
// whose class_name == label and score >= min_conf, or nullptr if none match.
// The nodes that need the actual detection (its bbox, center, size) all
// hand-rolled this same loop; fetch the array once and call this on that one
// snapshot (see contains_label's note on pairing presence with the stamp).
template <class Arr>
auto const* best_detection(Arr const& arr, std::string const& label, double min_conf)
{
    using Det = typename std::decay_t<decltype(arr.detections)>::value_type;
    Det const* best = nullptr;
    double best_conf = -1.0;
    for (auto const& d : arr.detections)
    {
        if (d.class_name == label && d.score >= min_conf && d.score > best_conf)
        {
            best = &d;
            best_conf = d.score;
        }
    }
    return best;
}

}  // namespace detection_gate
