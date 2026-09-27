#pragma once

#include <behaviortree_cpp/basic_types.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <string_view>

#include "prop_maneuvers/geometry.hpp"

namespace prop_mission_planner
{

/// A mission's reference to an object it is working on.
///
/// Today every reference is a STAND-IN: a point in the map frame, resolved by
/// locking onto the nearest lidar cluster. When the boat has a map with stable
/// object IDs this gains an ID and resolves through the map instead -- see
/// docs/superpowers/specs/2026-09-26-boat-map-transition.md. Missions hold a
/// reference, never their own copy of an object's coordinates.
struct ObjectRef
{
    prop_maneuvers::Point point;
};

namespace detail
{
/// One finite number with optional surrounding whitespace, and nothing else.
inline std::optional<double> parse_number(std::string_view text)
{
    std::string const copy(text);
    char const *begin = copy.c_str();
    char *end = nullptr;
    errno = 0;
    double const value = std::strtod(begin, &end);
    if (end == begin || errno == ERANGE || !std::isfinite(value))
    {
        return std::nullopt;
    }
    while (*end == ' ' || *end == '\t')
    {
        ++end;
    }
    if (*end != '\0')
    {
        return std::nullopt;
    }
    return value;
}
}  // namespace detail

/// "x;y" in the map frame. Nothing for any other text.
inline std::optional<ObjectRef> parse_object_ref(std::string_view text)
{
    auto const split = text.find(';');
    if (split == std::string_view::npos || text.find(';', split + 1) != std::string_view::npos)
    {
        return std::nullopt;
    }
    auto const x = detail::parse_number(text.substr(0, split));
    auto const y = detail::parse_number(text.substr(split + 1));
    if (!x || !y)
    {
        return std::nullopt;
    }
    return ObjectRef{ prop_maneuvers::Point{ *x, *y } };
}

/// Millimetre precision is far finer than any lock can use.
inline std::string to_string(ObjectRef const &ref)
{
    char text[64];
    std::snprintf(text, sizeof(text), "%.3f;%.3f", ref.point.x, ref.point.y);
    return text;
}

}  // namespace prop_mission_planner

namespace BT
{

template <>
[[nodiscard]] inline prop_mission_planner::ObjectRef convertFromString<prop_mission_planner::ObjectRef>(StringView str)
{
    auto const ref = prop_mission_planner::parse_object_ref(str);
    if (!ref)
    {
        throw RuntimeError("not an object reference: \"", std::string(str),
                           "\" (expected \"x;y\" in the map frame, e.g. \"-4.0;-4.0\")");
    }
    return *ref;
}

template <>
[[nodiscard]] inline std::string toStr<prop_mission_planner::ObjectRef>(prop_mission_planner::ObjectRef const &ref)
{
    return prop_mission_planner::to_string(ref);
}

}  // namespace BT
