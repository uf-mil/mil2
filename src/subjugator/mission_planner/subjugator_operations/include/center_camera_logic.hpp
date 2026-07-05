#pragma once

#include <cstdint>

// Pure logic for CenterCamera (unit-tested without ROS/BT), following the
// select_target_logic.hpp pattern.
namespace center_camera
{

// Distinguishes "target lost" from a transient detection flicker by counting
// consecutive *fresh* detection frames that lack the target. One glinted or
// occluded frame must not abort (or silently skip, via a wrapping Fallback)
// the whole centering behavior.
//
// Freshness is judged by the detection array's stamp: a frame is fresh when
// its stamp is newer than the last frame considered, so re-reads of the same
// array on fast BT ticks never advance the counter — `miss_frames` means
// "N distinct camera observations agree it's gone", independent of tick rate.
// stamp==0 (publisher not stamping) is always treated as fresh so the gate
// never stalls; the count then degrades to per-tick (documented legacy case).
struct MissGate
{
    std::int64_t last_seen_ns{ -1 };  // stamp of the last frame considered
    int misses{ 0 };                  // consecutive fresh frames without the target

    enum class Verdict
    {
        kStale,  // same frame as last time -> caller should hold (RUNNING)
        kHit,    // fresh frame, target present -> caller may act
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
            misses = 0;
            return Verdict::kHit;
        }
        ++misses;
        return misses >= miss_frames ? Verdict::kLost : Verdict::kMiss;
    }

    void reset()
    {
        last_seen_ns = -1;
        misses = 0;
    }
};

}  // namespace center_camera
