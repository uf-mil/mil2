/**
 * @file constants.hpp
 * @brief Declares and caches every prop_maneuvers tunable from one settings
 *        file (config/maneuvers.yaml). Mirrors pcd/pcd_constants.hpp.
 *
 * Inherit alongside rclcpp::Node:
 *   class FaceObject : public rclcpp::Node, public prop_maneuvers::Constants
 *   { FaceObject() : rclcpp::Node("face_object"), Constants(this) {} };
 */

#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_maneuvers/geometry.hpp"

namespace prop_maneuvers
{

class Constants
{
  public:
    explicit Constants(rclcpp::Node *node)
    {
        auto const deg = [](double d) { return d * M_PI / 180.0; };

        // ── Hull and sensor geometry ──────────────────────────────────────
        // MEASURED on the boat 2026-09-08, from the lidar's edge and converted
        // to its centre with the lidar's 4 7/8 in diameter. prop.urdf puts the
        // lidar directly above base_link, so they are base_link-relative as
        // these need to be. See config/maneuvers.yaml for the tape figures.
        //
        // base_link is NOT in the middle of the boat: 0.760 m to the front and
        // 0.367 m to the propellers. Two extents, because the boat meets things
        // differently by direction -- passing something to the side is bounded
        // by the hull's width, backing into something by how far it reaches
        // behind base_link.
        //
        // hull_behind is to the PROPELLERS; the pontoons reach further back, so
        // it is optimistic for reversing until the tails are measured.
        hull_half_width_ = node->declare_parameter("hull_half_width", 0.443);
        hull_behind_ = node->declare_parameter("hull_behind", 0.367);
        hull_front_ = node->declare_parameter("hull_front", 0.760);
        guidance_hold_radius_ = node->declare_parameter("guidance_hold_radius", 1.0);
        auto const spots = node->declare_parameter("blind_spots_deg", std::vector<double>{});
        use_blind_spots_ = node->declare_parameter("use_blind_spots", true);
        // Refuse to start on a malformed list rather than quietly carrying on
        // with no blind spots. An odd-length list is always a typo, and the
        // quiet path is the dangerous one: the Task 10 mask would then mask
        // nothing, and the blind-spot test would pass without testing anything.
        if (spots.size() % 2 != 0)
        {
            RCLCPP_FATAL(node->get_logger(), "blind_spots_deg needs a flat list of from, to pairs; got %zu values.",
                         spots.size());
            throw std::runtime_error("blind_spots_deg must hold an even number of values");
        }
        for (std::size_t i = 0; i + 1 < spots.size(); i += 2)
        {
            // Each pair is read anticlockwise from `from` to `to`, so a pair
            // given the wrong way round does not describe a narrow wedge -- it
            // describes everything EXCEPT that wedge. Swapping 90 and 135 turns
            // a 45 degree blind spot into a 315 degree one and the mask blanks
            // almost the whole scan, which looks like a dead lidar rather than
            // a typo. Same reasoning as the odd-length check above: refuse
            // rather than run on a list that cannot mean what was intended.
            //
            // A pair is legitimate up to 180 degrees; wider than that and the
            // sighted arc is the smaller one, which is certainly not a "spot".
            double const span = wrap_angle(deg(spots[i + 1]) - deg(spots[i]));
            if (span <= 0.0)
            {
                RCLCPP_FATAL(node->get_logger(),
                             "blind_spots_deg pair (%.1f, %.1f) runs backwards; each pair goes anticlockwise from "
                             "the first value to the second.",
                             spots[i], spots[i + 1]);
                throw std::runtime_error("blind_spots_deg pairs must run anticlockwise from, to");
            }
            blind_spots_.push_back(BlindSpot{ deg(spots[i]), deg(spots[i + 1]) });
        }

        // ── Acquiring ─────────────────────────────────────────────────────
        acquire_cone_ = deg(node->declare_parameter("acquire_cone_deg", 45.0));
        acquire_max_range_ = node->declare_parameter("acquire_max_range", 30.0);

        // ── Matching ──────────────────────────────────────────────────────
        max_object_radius_ = node->declare_parameter("max_object_radius", 1.5);
        match_radius_ = node->declare_parameter("match_radius", 3.0);
        refresh_max_jump_ = node->declare_parameter("refresh_max_jump", 1.0);
        ambiguous_margin_ = node->declare_parameter("ambiguous_margin", 1.0);
        reading_max_age_ = node->declare_parameter("reading_max_age", 5.0);

        // ── Distances ─────────────────────────────────────────────────────
        circle_radius_ = node->declare_parameter("circle_radius", 6.0);
        circle_legs_ = static_cast<int>(node->declare_parameter("circle_legs", 4));
        circle_counter_clockwise_ = node->declare_parameter("circle_counter_clockwise", true);
        approach_standoff_ = node->declare_parameter("approach_standoff", 1.0);
        min_gap_ = node->declare_parameter("min_gap", 0.5);
        detour_clearance_ = node->declare_parameter("detour_clearance", 0.5);
        target_blob_margin_ = node->declare_parameter("target_blob_margin", 1.0);

        // ── Reversing ─────────────────────────────────────────────────────
        reverse_speed_ = node->declare_parameter("reverse_speed", 0.4);
        reverse_max_distance_ = node->declare_parameter("reverse_max_distance", 2.0);

        // ── Tolerances ────────────────────────────────────────────────────
        arrive_tolerance_ = node->declare_parameter("arrive_tolerance", 1.5);
        standoff_tolerance_ = node->declare_parameter("standoff_tolerance", 0.3);
        corner_tolerance_ = node->declare_parameter("corner_tolerance", 0.5);
        point_tolerance_ = deg(node->declare_parameter("point_tolerance_deg", 5.0));

        // ── Turning ───────────────────────────────────────────────────────
        turn_gain_ = node->declare_parameter("turn_gain", 1.2);
        max_turn_rate_ = node->declare_parameter("max_turn_rate", 0.6);

        // ── Giving up ─────────────────────────────────────────────────────
        maneuver_timeout_ = node->declare_parameter("maneuver_timeout", 300.0);
    }

    // Public, deliberately. The sibling pattern in pcd_constants.hpp keeps its
    // cached values protected because only derived nodes read them. Here the
    // helper classes -- TargetLock, Context -- hold a `Constants const &`
    // WITHOUT inheriting it, so protected would not compile.
    // The YAML in config/maneuvers.yaml and the defaults below must be kept
    // identical; nothing enforces that automatically.
    double hull_half_width_{ 0.0 };
    double hull_behind_{ 0.0 };
    double hull_front_{ 0.0 };
    double guidance_hold_radius_{ 0.0 };
    std::vector<BlindSpot> blind_spots_;
    bool use_blind_spots_{ true };
    double acquire_cone_{ 0.0 };
    double acquire_max_range_{ 0.0 };
    double max_object_radius_{ 0.0 };
    double match_radius_{ 0.0 };
    double refresh_max_jump_{ 0.0 };
    double ambiguous_margin_{ 0.0 };
    double reading_max_age_{ 0.0 };
    double circle_radius_{ 0.0 };
    int circle_legs_{ 4 };
    bool circle_counter_clockwise_{ true };
    double approach_standoff_{ 0.0 };
    double min_gap_{ 0.0 };
    double detour_clearance_{ 0.0 };
    double target_blob_margin_{ 0.0 };
    double reverse_speed_{ 0.0 };
    double reverse_max_distance_{ 0.0 };
    double arrive_tolerance_{ 0.0 };
    double standoff_tolerance_{ 0.0 };
    double corner_tolerance_{ 0.0 };
    double point_tolerance_{ 0.0 };
    double turn_gain_{ 0.0 };
    double max_turn_rate_{ 0.0 };
    double maneuver_timeout_{ 0.0 };
};

}  // namespace prop_maneuvers
