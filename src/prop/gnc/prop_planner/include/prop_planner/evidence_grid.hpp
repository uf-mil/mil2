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
/// A frame that hits a cell adds hit_gain; a frame that should have seen it,
/// within verify_range of the lidar, and did not takes miss_loss. A buoy
/// saturates in a few frames, a wave never gets far, and a buoy that drifts
/// leaves no trail because the cells it vacates are debited. Nothing beyond
/// verify_range is forgotten, so what was seen astern stays on the map.
class EvidenceGrid
{
  public:
    struct Config
    {
        double resolution{ 0.25 };  ///< m per cell
        double size{ 80.0 };        ///< m, the window's edge
        int hit_gain{ 25 };
        int miss_loss{ 10 };
        double verify_range{ 12.0 };  ///< m, within which not being seen counts
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

    Config config_;
    Grid grid_;
    bool placed_{ false };
    std::vector<uint8_t> hit_this_frame_;
    std::vector<std::size_t> touched_;
};

}  // namespace prop_planner
