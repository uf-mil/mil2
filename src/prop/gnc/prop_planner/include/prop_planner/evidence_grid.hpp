#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "prop_planner/grid.hpp"

namespace prop_planner
{

/// A window of cells that follows the boat, each holding 0..100 of evidence
/// that something solid stands there.
///
/// A frame that hits a cell adds hit_gain; a frame that should have seen it
/// and did not takes miss_loss. A buoy saturates in a few frames, a wave never
/// gets far, and a buoy that drifts leaves no trail because the cells it
/// vacates are debited.
///
/// Should have seen it means from min_range out to verify_range, with nothing
/// solid in the way. Every other cell keeps its evidence, so a buoy hidden
/// behind another stays on the map, and so does what was seen astern.
class EvidenceGrid
{
  public:
    struct Config
    {
        double resolution{ 0.25 };  ///< m per cell
        double size{ 80.0 };        ///< m, the window's edge
        int hit_gain{ 25 };
        int miss_loss{ 10 };
        double verify_range{ 12.0 };  ///< m, out to which not being seen counts
        double min_range{ 0.0 };      ///< m, inside which not being seen does not count
        int occupied{ 60 };           ///< evidence at or above this hides what is behind it
    };

    explicit EvidenceGrid(Config config);

    /// Keep the boat near the middle, shifting the window when it strays.
    void follow(Point boat);

    /// Fold in one lidar frame: where it hit and where it looked from, both in
    /// the grid's frame.
    void integrate(std::vector<Point> const& hits, Point sensor);

    void clear();

    Grid const& grid() const
    {
        return grid_;
    }

  private:
    void shift(int dx, int dy);
    void add_hits(std::vector<Point> const& hits);
    void fade_unseen(Point sensor);
    bool hidden(Cell cell, Point sensor) const;

    Config config_;
    Grid grid_;
    bool placed_{ false };
    std::vector<uint8_t> hit_this_frame_;
    std::vector<std::size_t> touched_;
    std::vector<std::size_t> fading_;
};

}  // namespace prop_planner
