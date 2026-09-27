#include "prop_mission_planner/maneuver_nodes.hpp"

#include <cmath>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "prop_mission_planner/context.hpp"
#include "prop_mission_planner/ports.hpp"

namespace prop_mission_planner
{

namespace
{
/// true for counter-clockwise, as prop_maneuvers::CircleObject wants it.
/// Nothing for any other text.
std::optional<bool> counter_clockwise_from(std::string const &direction)
{
    if (direction == "clockwise")
    {
        return false;
    }
    if (direction == "counter_clockwise")
    {
        return true;
    }
    return std::nullopt;
}

bool valid_radius(double radius)
{
    return std::isfinite(radius) && radius > 0.0;
}

/// A ring of fewer than three legs is a line, not a circle.
constexpr int kMinLegs{ 3 };

bool valid_standoff(double standoff)
{
    return std::isfinite(standoff) && standoff >= 0.0;
}
}  // namespace

BT::PortsList FaceObject::providedPorts()
{
    return common_ports();
}

std::unique_ptr<prop_maneuvers::Maneuver> FaceObject::make(prop_maneuvers::Context &maneuvers)
{
    return std::make_unique<prop_maneuvers::FaceObject>(maneuvers);
}

ApproachObject::ApproachObject(std::string const &name, BT::NodeConfig const &config) : ManeuverNode(name, config)
{
    if (auto const literal = literal_port(config, "standoff"))
    {
        if (!valid_standoff(BT::convertFromString<double>(*literal)))
        {
            throw BT::RuntimeError(name, ": standoff must be a finite number of metres >= 0, not \"", *literal, "\"");
        }
    }
}

BT::PortsList ApproachObject::providedPorts()
{
    auto ports = common_ports();
    ports.insert(BT::InputPort<double>("standoff", "Metres from the bow to the object's surface. Default: "
                                                   "approach_standoff in maneuvers.yaml"));
    return ports;
}

std::unique_ptr<prop_maneuvers::Maneuver> ApproachObject::make(prop_maneuvers::Context &maneuvers)
{
    auto const logger = maneuvers.node->get_logger();
    double standoff = maneuvers.settings.approach_standoff_;
    if (port_given(config(), "standoff"))
    {
        auto const given = getInput<double>("standoff");
        if (!given)
        {
            RCLCPP_ERROR(logger, "%s: standoff: %s", name().c_str(), given.error().c_str());
            return nullptr;
        }
        standoff = *given;
    }
    if (!valid_standoff(standoff))
    {
        RCLCPP_ERROR(logger, "%s: standoff must be >= 0 m, got %.2f", name().c_str(), standoff);
        return nullptr;
    }
    return std::make_unique<prop_maneuvers::ApproachObject>(maneuvers, standoff);
}

CircleObject::CircleObject(std::string const &name, BT::NodeConfig const &config) : ManeuverNode(name, config)
{
    // No default on purpose: a mission that forgets the direction fails to
    // load instead of quietly taking maneuvers.yaml's counter-clockwise.
    if (!port_given(config, "direction"))
    {
        throw BT::RuntimeError(name, ": direction is required (\"clockwise\" or \"counter_clockwise\")");
    }
    if (auto const literal = literal_port(config, "direction"))
    {
        if (!counter_clockwise_from(*literal))
        {
            throw BT::RuntimeError(name, ": direction must be \"clockwise\" or \"counter_clockwise\", not \"", *literal,
                                   "\"");
        }
    }
    if (auto const literal = literal_port(config, "radius"))
    {
        if (!valid_radius(BT::convertFromString<double>(*literal)))
        {
            throw BT::RuntimeError(name, ": radius must be a finite number of metres > 0, not \"", *literal, "\"");
        }
    }
    if (auto const literal = literal_port(config, "legs"))
    {
        if (BT::convertFromString<int>(*literal) < kMinLegs)
        {
            throw BT::RuntimeError(name, ": legs must be at least ", std::to_string(kMinLegs), ", not \"", *literal,
                                   "\"");
        }
    }
}

BT::PortsList CircleObject::providedPorts()
{
    auto ports = common_ports();
    ports.insert(BT::InputPort<std::string>("direction", "\"clockwise\" or \"counter_clockwise\" (required)"));
    ports.insert(BT::InputPort<double>("radius", "Metres. Default: circle_radius in maneuvers.yaml"));
    ports.insert(BT::InputPort<int>("legs", "Straight legs per lap. Default: circle_legs in maneuvers.yaml"));
    return ports;
}

std::unique_ptr<prop_maneuvers::Maneuver> CircleObject::make(prop_maneuvers::Context &maneuvers)
{
    auto const logger = maneuvers.node->get_logger();
    auto const &settings = maneuvers.settings;

    auto const direction = getInput<std::string>("direction");
    if (!direction)
    {
        RCLCPP_ERROR(logger, "%s: direction: %s", name().c_str(), direction.error().c_str());
        return nullptr;
    }
    auto const counter_clockwise = counter_clockwise_from(*direction);
    if (!counter_clockwise)
    {
        RCLCPP_ERROR(logger, "%s: direction must be \"clockwise\" or \"counter_clockwise\", not \"%s\"", name().c_str(),
                     direction->c_str());
        return nullptr;
    }

    // radius and legs have no port default, so "given" is detectable: an
    // unreadable {blackboard} entry fails rather than quietly using the yaml.
    double radius = settings.circle_radius_;
    if (port_given(config(), "radius"))
    {
        auto const given = getInput<double>("radius");
        if (!given)
        {
            RCLCPP_ERROR(logger, "%s: radius: %s", name().c_str(), given.error().c_str());
            return nullptr;
        }
        radius = *given;
    }
    int legs = settings.circle_legs_;
    if (port_given(config(), "legs"))
    {
        auto const given = getInput<int>("legs");
        if (!given)
        {
            RCLCPP_ERROR(logger, "%s: legs: %s", name().c_str(), given.error().c_str());
            return nullptr;
        }
        legs = *given;
    }
    if (!valid_radius(radius) || legs < kMinLegs)
    {
        RCLCPP_ERROR(logger, "%s: need radius > 0 m and legs >= %d, got radius %.2f m, legs %d", name().c_str(),
                     kMinLegs, radius, legs);
        return nullptr;
    }

    // The boat drives straight legs between corners ON the ring, so it passes
    // closest to the object mid-leg, at radius * cos(pi / legs), not at the
    // radius itself. That must clear the hull from the object's surface by the
    // same amount the approach's obstacle check demands before it leaves an
    // object alone (plan_detour's trigger radius in prop_maneuvers
    // geometry.cpp: obstacle radius + hull_half_width + min_gap).
    double const closest = radius * std::cos(M_PI / legs);
    double const needed = maneuvers.lock.radius() + settings.hull_half_width_ + settings.min_gap_;
    if (closest < needed)
    {
        RCLCPP_ERROR(logger,
                     "%s: a %.2f m ring of %d legs passes %.2f m from the object's centre; the hull needs %.2f m "
                     "(object radius %.2f + hull half width %.2f + min_gap %.2f)",
                     name().c_str(), radius, legs, closest, needed, maneuvers.lock.radius(), settings.hull_half_width_,
                     settings.min_gap_);
        return nullptr;
    }
    return std::make_unique<prop_maneuvers::CircleObject>(maneuvers, radius, legs, *counter_clockwise);
}

// Last, after everything the nodes above use (see REGISTER in context.hpp).
REGISTER(FaceObject)
REGISTER(ApproachObject)
REGISTER(CircleObject)

}  // namespace prop_mission_planner
