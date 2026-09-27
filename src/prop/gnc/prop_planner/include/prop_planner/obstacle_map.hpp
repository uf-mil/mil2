#pragma once

#include <cstddef>
#include <vector>

#include "prop_planner/geometry.hpp"

namespace prop_planner
{

/// One remembered obstacle, in whatever frame the map was fed in.
struct Obstacle
{
    Capsule shape;
    int hits{ 0 };  ///< observations folded into this entry
};

/// Fold an observation into a capsule that already exists.
///
/// Three things make this more than a blend:
///
/// A segment can arrive either way round, and lerping a flipped observation
/// endpoint-to-endpoint drags both ends together and collapses the capsule to
/// a point, so the pairing is chosen first.
///
/// Extent accumulates rather than averaging. Lidar sees part of an extended
/// object, so each view is a slice of the true shape; averaging endpoints
/// would shrink the capsule to whichever slice is in view and let it oscillate
/// as the boat drives past. Direction, lateral position and radius do average,
/// because those are observed whole every frame.
///
/// Growth past the stored ends needs a real reach, more than extent_deadband,
/// or observation jitter would slowly smear a round buoy into a capsule.
Capsule merge_capsules(Capsule stored, Capsule observation, double gain, double extent_deadband);

// A persistent, sparse record of where the obstacles are.
//
// pcl_tracker reports what the lidar can see this instant and deletes a track
// five missed frames after it leaves the beam. That is the right policy for a
// tracker and useless for a planner, which has to route around the buoy it
// passed thirty seconds ago. This is the memory: detections go in one at a
// time, and what comes out is everything seen so far.
//
// Entries are associated by position, not by the tracker's IDs. Those IDs are
// stable only while an object stays in view - a buoy that leaves the beam and
// comes back arrives with a new one - so they cannot key a map meant to
// outlive the observation.
//
// Nothing is ever forgotten. Over a course that takes minutes to drive, a buoy
// seen once is still there later, and the min_hits gate already keeps wave
// crests and other single-frame phantoms out. If a run ever gets long enough
// for stale entries to matter, the honest fix is negative information - decay
// an entry only when it was inside the sensor's range and field of view and
// still went unseen - rather than a timeout, which would forget the buoys
// behind the boat just as reliably as the ones that were never there.
class ObstacleMap
{
  public:
    struct Config
    {
        /// Two observations closer together than this are the same object.
        /// Measured between axes, not centres: see one end of a dock and then
        /// the other and the centres are far apart though the shapes overlap.
        double merge_distance{ 2.0 };
        /// Weight given to a new observation when folding it into an existing
        /// entry. Small means a steady map that trusts its own history.
        double position_gain{ 0.2 };
        /// Observations before an entry is worth routing around.
        int min_hits{ 3 };
        /// Largest radius any one entry may claim, so a frame that merges two
        /// buoys into one blob cannot wall off the course.
        double max_radius{ 5.0 };
        /// Largest length any one entry may claim. Extent only ever grows, so
        /// this is the valve on a bad association welding two objects into one
        /// long capsule - the safety a convex hull cannot offer, because a
        /// polygon has no single number to clamp.
        double max_length{ 20.0 };
        /// How far past a stored end an observation has to reach before it
        /// counts as revealing more of the object rather than as noise.
        double extent_deadband{ 0.3 };
        /// Hard cap on entries. Past it, the least-seen entry is evicted.
        std::size_t capacity{ 256 };
    };

    explicit ObstacleMap(Config config) : config_(config)
    {
    }

    /// Fold one detection into the map, merging it with a nearby entry if
    /// there is one and inserting a new entry otherwise.
    void observe(Capsule observation);

    /// The entries seen often enough to be worth planning around.
    std::vector<Obstacle> confirmed() const;

    /// Whether an entry has been seen often enough to be planned around. The
    /// ones that have not are still worth looking at: a buoy that never gets
    /// past this line is being seen, but not often enough or not in a
    /// consistent enough place, which is a different problem from not being
    /// seen at all.
    bool is_confirmed(Obstacle const& obstacle) const
    {
        return obstacle.hits >= config_.min_hits;
    }

    std::vector<Obstacle> const& all() const
    {
        return obstacles_;
    }

  private:
    /// Index of the entry whose axis is closest within merge_distance, or -1.
    int nearest(Capsule const& observation) const;
    void evict_least_seen();
    /// Trim a capsule back inside max_radius and max_length.
    Capsule clamped(Capsule capsule) const;

    Config config_;
    std::vector<Obstacle> obstacles_;
};

}  // namespace prop_planner
