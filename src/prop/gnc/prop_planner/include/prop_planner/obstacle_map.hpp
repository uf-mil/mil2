#pragma once

#include <cstddef>
#include <vector>

#include "prop_planner/geometry.hpp"

namespace prop_planner
{

struct Obstacle
{
    Capsule shape;
    int hits{ 0 };
    int misses{ 0 };     ///< consecutive frames it should have been seen and was not
    bool seen{ false };  ///< matched since the last forget_unseen
};

/// Fold an observation into an existing capsule.
///
/// Endpoint pairing is chosen first, or a segment that arrived reversed drags
/// both ends together and collapses the capsule. An observation longer than
/// extent_deadband is a slice of the object, so its ends accumulate; a shorter
/// one carries position only and slides the shape instead.
Capsule merge_capsules(Capsule stored, Capsule observation, double gain, double extent_deadband);

/// A persistent, sparse record of where the obstacles are.
///
/// pcl_tracker drops a track five missed frames after it leaves the beam, which
/// is right for a tracker and useless for a planner. This is the memory.
/// Entries associate by position rather than by track ID, since those IDs do
/// not survive an object leaving the beam and coming back.
class ObstacleMap
{
  public:
    struct Config
    {
        double merge_distance{ 1.5 };
        double position_gain{ 0.2 };
        int min_hits{ 3 };
        double max_radius{ 5.0 };
        double max_length{ 20.0 };
        double extent_deadband{ 0.3 };
        /// Entries within this of the sensor are close enough that failing to
        /// see them counts against them. Wants to be well inside the range
        /// where detection is reliable, or real obstacles get forgotten.
        double verify_range{ 15.0 };
        /// Consecutive unseen-but-should-have-been frames before an entry goes.
        int max_misses{ 15 };
        std::size_t capacity{ 256 };
    };

    explicit ObstacleMap(Config config) : config_(config)
    {
    }

    /// Fold one detection into the map.
    void observe(Capsule observation);

    /// Drop entries that were close enough to be seen and were not. Call once
    /// per frame, after that frame's observations, with the sensor's position.
    ///
    /// Range is the gate rather than a timeout: a timeout forgets the buoys
    /// astern just as fast as the ones that were never there.
    void forget_unseen(Point sensor);

    std::vector<Obstacle> confirmed() const;

    bool is_confirmed(Obstacle const& obstacle) const
    {
        return obstacle.hits >= config_.min_hits;
    }

    std::vector<Obstacle> const& all() const
    {
        return obstacles_;
    }

    void clear()
    {
        obstacles_.clear();
    }

  private:
    int nearest(Capsule const& observation) const;
    void evict_least_seen();
    Capsule clamped(Capsule capsule) const;

    Config config_;
    std::vector<Obstacle> obstacles_;
};

}  // namespace prop_planner
