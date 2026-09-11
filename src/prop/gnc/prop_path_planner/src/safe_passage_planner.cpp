#include "prop_path_planner/safe_passage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

namespace prop_controller
{
namespace
{
double dot(Point a, Point b)
{
    return a.x * b.x + a.y * b.y;
}  // dot product, if pos == same direction ;0 == perp; neg == opposite
double norm(Point v)
{
    return std::hypot(v.x, v.y);
}  // length of vector

Point operator-(Point a, Point b)
{
    return { a.x - b.x, a.y - b.y };
}
Point operator+(Point a, Point b)
{
    return { a.x + b.x, a.y + b.y };
}
Point operator*(Point v, double s)
{
    return { v.x * s, v.y * s };
}

Point normalize(Point v)
{
    double const n = norm(v);
    return n > 1e-9 ? Point{ v.x / n, v.y / n } : Point{ 1.0, 0.0 };
}

// needed to offset a waypoint to the left or right of a buoy
Point rotate90(Point v, bool clockwise)
{
    return clockwise ? Point{ v.y, -v.x } : Point{ -v.y, v.x };
}

// shortest distance from p to a-b
// used to check whether a black buoy is too close to a leg of the path.
double distanceToSegment(Point p, Point a, Point b)
{
    Point const ab = b - a;
    double const len2 = dot(ab, ab);
    double t = len2 > 1e-9 ? dot(p - a, ab) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    Point const closest = a + ab * t;
    return norm(p - closest);
}

// Points around a circle of radius centered at center, starting at the
// point closest to approach_from and sweeping one full loop in the given
// direction. The first and last points are the same, closing the loop.
std::vector<Point> circleWaypoints(Point center, double radius, Point approach_from, bool clockwise, int segments)
{
    double const start_angle = std::atan2(approach_from.y - center.y, approach_from.x - center.x);
    double const step = (clockwise ? -1.0 : 1.0) * (2.0 * M_PI / segments);

    std::vector<Point> points;
    for (int i = 0; i <= segments; ++i)
    {
        double const angle = start_angle + step * i;
        points.push_back({ center.x + radius * std::cos(angle), center.y + radius * std::sin(angle) });
    }
    return points;
}  // namespace

}  // namespace

std::vector<Point> planSafePassage(std::vector<Buoy> const& buoys, Point boat_start, PlannerParams const& params)
{
    Buoy const* entry = nullptr;
    Buoy const* exit_buoy = nullptr;
    std::vector<Buoy const*> gates;  // red, green, and black buoys to thread through

    for (auto const& buoy : buoys)
    {
        switch (buoy.color)
        {
            case BuoyColor::FLASH_BLUE:
                entry = &buoy;
                break;
            case BuoyColor::SOLID_BLUE:
                exit_buoy = &buoy;
                break;
            default:
                gates.push_back(&buoy);
                break;
        }
    }

    if (entry == nullptr || exit_buoy == nullptr)
    {
        throw std::runtime_error("planSafePassage needs exactly one FLASH_BLUE (entry) and one SOLID_BLUE (exit) buoy");
    }

    // This computes the unit vector pointing from entry to exit
    // and two perpendicular directions off of it to offset the waypoints
    Point const axis = normalize(exit_buoy->position - entry->position);
    Point const left = rotate90(axis, false);  // 90 deg counterclockwise from axis
    Point const right = rotate90(axis, true);  // 90 deg clockwise from axis

    // order the gate buoys by how far along the axis they sit, so we thread
    // through them in the order the boat will actually reach them, instead of
    // whatever order they were detected in.
    std::sort(gates.begin(), gates.end(), [&](Buoy const* a, Buoy const* b)
              { return dot(a->position - entry->position, axis) < dot(b->position - entry->position, axis); });

    std::vector<Point> path;

    // circle the entry buoy clockwise, starting from wherever the boat currently is, and closing back at that same
    // point.
    auto const entry_circle =
        circleWaypoints(entry->position, params.circle_radius, boat_start, /*clockwise=*/true, params.circle_segments);
    path.insert(path.end(), entry_circle.begin(), entry_circle.end());

    // Thread the red/green buoys, offsetting each one to the correct side.
    // Black buoys don't get their own waypoint, they're only checked against the straight legs below.
    for (auto const* buoy : gates)
    {
        if (buoy->color == BuoyColor::RED)
        {
            // Pass on the starboard side -> aim to the left of the buoy.
            path.push_back(buoy->position + left * params.pass_offset);
        }
        else if (buoy->color == BuoyColor::GREEN)
        {
            // Pass on the port side -> aim to the right of the buoy.
            path.push_back(buoy->position + right * params.pass_offset);
        }
    }

    // Circle the exit buoy counterclockwise, starting from wherever the path currently ends.
    auto const exit_circle = circleWaypoints(exit_buoy->position, params.circle_radius, path.back(),
                                             /*clockwise=*/false, params.circle_segments);
    path.insert(path.end(), exit_circle.begin(), exit_circle.end());

    // Safety pass: nudge any leg that comes too close to a black buoy.
    //  one dodge point per buoy should be enough, so we stop after
    //  inserting the first one and move to the next buoy.
    for (auto const* buoy : gates)
    {
        if (buoy->color != BuoyColor::BLACK)
        {
            continue;
        }

        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            if (distanceToSegment(buoy->position, path[i], path[i + 1]) >= params.safety_margin)
            {
                continue;
            }

            bool const lean_right = dot(buoy->position - entry->position, right) > 0.0;
            Point const dodge = buoy->position + (lean_right ? left : right) * params.safety_margin;
            path.insert(path.begin() + i + 1, dodge);
            break;
        }
    }

    return path;
}

}  // namespace prop_controller

// ROS entry point.  Buoys are supplied as parameters so this node can also be
// used by mission code which has not yet been migrated to a buoy topic.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("safe_passage_planner");

    auto const x = node->declare_parameter<std::vector<double>>("buoy_x", {});
    auto const y = node->declare_parameter<std::vector<double>>("buoy_y", {});
    auto const colors = node->declare_parameter<std::vector<std::string>>("buoy_colors", {});
    auto const boat_x = node->declare_parameter<double>("boat_start_x", 0.0);
    auto const boat_y = node->declare_parameter<double>("boat_start_y", 0.0);
    auto const frame = node->declare_parameter<std::string>("frame_id", "map");

    if (x.size() != y.size() || x.size() != colors.size())
    {
        RCLCPP_FATAL(node->get_logger(), "buoy_x, buoy_y, and buoy_colors must have equal lengths");
        rclcpp::shutdown();
        return 1;
    }

    std::vector<prop_controller::Buoy> buoys;
    for (std::size_t i = 0; i < x.size(); ++i)
    {
        prop_controller::Buoy buoy;
        buoy.position = { x[i], y[i] };
        if (colors[i] == "FLASH_BLUE")
            buoy.color = prop_controller::BuoyColor::FLASH_BLUE;
        else if (colors[i] == "SOLID_BLUE")
            buoy.color = prop_controller::BuoyColor::SOLID_BLUE;
        else if (colors[i] == "RED")
            buoy.color = prop_controller::BuoyColor::RED;
        else if (colors[i] == "GREEN")
            buoy.color = prop_controller::BuoyColor::GREEN;
        else if (colors[i] == "BLACK")
            buoy.color = prop_controller::BuoyColor::BLACK;
        else
        {
            RCLCPP_FATAL(node->get_logger(), "unknown buoy color: %s", colors[i].c_str());
            rclcpp::shutdown();
            return 1;
        }
        buoys.push_back(buoy);
    }

    auto publisher = node->create_publisher<nav_msgs::msg::Path>("safe_passage_path", 10);
    try
    {
        auto const points = prop_controller::planSafePassage(buoys, { boat_x, boat_y }, {});
        nav_msgs::msg::Path message;
        message.header.frame_id = frame;
        for (auto const& point : points)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = message.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.orientation.w = 1.0;
            message.poses.push_back(pose);
        }
        publisher->publish(message);
    }
    catch (std::exception const& error)
    {
        RCLCPP_FATAL(node->get_logger(), "failed to plan safe passage: %s", error.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
