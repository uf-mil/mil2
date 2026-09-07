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
            blind_spots_.push_back(BlindSpot{ deg(spots[i]), deg(spots[i + 1]) });
        }

        // ── Acquiring ─────────────────────────────────────────────────────
        acquire_cone_ = deg(node->declare_parameter("acquire_cone_deg", 45.0));
        acquire_max_range_ = node->declare_parameter("acquire_max_range", 30.0);

        // ── Matching ──────────────────────────────────────────────────────
        match_radius_ = node->declare_parameter("match_radius", 3.0);
        ambiguous_margin_ = node->declare_parameter("ambiguous_margin", 1.0);
        reading_max_age_ = node->declare_parameter("reading_max_age", 5.0);

        // ── Distances ─────────────────────────────────────────────────────
        circle_radius_ = node->declare_parameter("circle_radius", 6.0);
        circle_legs_ = static_cast<int>(node->declare_parameter("circle_legs", 4));
        circle_counter_clockwise_ = node->declare_parameter("circle_counter_clockwise", true);
        approach_standoff_ = node->declare_parameter("approach_standoff", 3.0);
        detour_clearance_ = node->declare_parameter("detour_clearance", 2.0);

        // ── Tolerances ────────────────────────────────────────────────────
        arrive_tolerance_ = node->declare_parameter("arrive_tolerance", 1.5);
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
    std::vector<BlindSpot> blind_spots_;
    bool use_blind_spots_{ true };
    double acquire_cone_{ 0.0 };
    double acquire_max_range_{ 0.0 };
    double match_radius_{ 0.0 };
    double ambiguous_margin_{ 0.0 };
    double reading_max_age_{ 0.0 };
    double circle_radius_{ 0.0 };
    int circle_legs_{ 4 };
    bool circle_counter_clockwise_{ true };
    double approach_standoff_{ 0.0 };
    double detour_clearance_{ 0.0 };
    double arrive_tolerance_{ 0.0 };
    double point_tolerance_{ 0.0 };
    double turn_gain_{ 0.0 };
    double max_turn_rate_{ 0.0 };
    double maneuver_timeout_{ 0.0 };
};

}  // namespace prop_maneuvers
