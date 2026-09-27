#include "prop_planner/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace prop_planner
{
namespace
{

/// pcl_tracker leads every message with a DELETEALL sentinel to clear RViz.
/// Only the boxes after it are detections.
bool is_detection(visualization_msgs::msg::Marker const& marker)
{
    return marker.type == visualization_msgs::msg::Marker::CUBE &&
           marker.action == visualization_msgs::msg::Marker::ADD;
}

double yaw_of(geometry_msgs::msg::Quaternion const& q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternion_of(double yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
}

/// The smallest capsule that encloses an oriented box.
///
/// pcl_clustering fits each cluster's principal axis, so a detection arrives
/// as a rectangle L by W with a real heading rather than as a box aligned to
/// whichever way the boat happened to be pointing. The capsule that covers it
/// runs L - W along that heading with a radius of W / sqrt(2): the corners sit
/// exactly that far from the segment's ends, and a square footprint collapses
/// to its circumscribed circle, which is what a buoy should be.
Capsule capsule_of(geometry_msgs::msg::Point const& centre, geometry_msgs::msg::Vector3 const& scale, double yaw)
{
    double along = scale.x;
    double across = scale.y;
    if (across > along)
    {
        std::swap(along, across);
        yaw += 0.5 * M_PI;  // the long axis is the box's local y
    }

    double const half = 0.5 * std::max(0.0, along - across);
    double const dx = half * std::cos(yaw);
    double const dy = half * std::sin(yaw);

    return Capsule{ { centre.x - dx, centre.y - dy }, { centre.x + dx, centre.y + dy }, across / std::sqrt(2.0) };
}

}  // namespace

Planner::Planner()
  : Node("planner")
  , map_({ declare_parameter("merge_distance", 2.0), declare_parameter("position_gain", 0.2),
           static_cast<int>(declare_parameter("min_hits", 3)), declare_parameter("max_radius", 5.0),
           declare_parameter("max_length", 20.0), declare_parameter("extent_deadband", 0.3),
           static_cast<std::size_t>(declare_parameter("capacity", 256)) })
  , planner_({ declare_parameter("inflation", 2.0), static_cast<int>(declare_parameter("corners_per_obstacle", 8)) })
{
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    replan_threshold_ = declare_parameter("replan_threshold", 0.5);
    inflation_ = get_parameter("inflation").as_double();
    double const rate = declare_parameter("rate", 1.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tracks_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "tracked_markers", rclcpp::QoS(10),
        [this](visualization_msgs::msg::MarkerArray::SharedPtr const msg) { tracks_callback(*msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered/global", 10,
                                                             [this](nav_msgs::msg::Odometry::SharedPtr const msg)
                                                             {
                                                                 position_ = { msg->pose.pose.position.x,
                                                                               msg->pose.pose.position.y };
                                                                 located_ = true;
                                                             });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr const msg)
        {
            goal_ = { msg->pose.position.x, msg->pose.position.y };
            goal_orientation_ = msg->pose.orientation;
            has_goal_ = true;
            published_.clear();  // a new goal always deserves a fresh plan
            RCLCPP_INFO(get_logger(), "new goal at (%.1f, %.1f)", goal_.x, goal_.y);
        });

    // Guidance subscribes transient local, so a plan published before it is up
    // still reaches it.
    rclcpp::QoS latched(1);
    latched.transient_local();
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("plan", latched);
    obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_map", rclcpp::QoS(1));

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { replan(); });

    RCLCPP_INFO(get_logger(), "planner started - send a goal on goal_pose, or use RViz's 2D Goal Pose");
}

void Planner::tracks_callback(visualization_msgs::msg::MarkerArray const& msg)
{
    auto const first = std::find_if(msg.markers.begin(), msg.markers.end(), is_detection);
    if (first == msg.markers.end())
    {
        return;  // a bare DELETEALL: nothing was in view this frame
    }

    // Every detection in one message shares a frame and a stamp, so the
    // transform is looked up once here instead of once per detection.
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_->lookupTransform(map_frame_, first->header.frame_id, first->header.stamp,
                                                tf2::durationFromSec(0.1));
    }
    catch (tf2::TransformException const& ex)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no transform %s -> %s, dropping this frame: %s",
                             first->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
        return;
    }

    for (auto const& marker : msg.markers)
    {
        if (!is_detection(marker))
        {
            continue;
        }

        // The whole pose, so the box's heading comes across with its centre.
        geometry_msgs::msg::Pose pose;
        tf2::doTransform(marker.pose, pose, transform);

        map_.observe(capsule_of(pose.position, marker.scale, yaw_of(pose.orientation)));
    }
}

void Planner::replan()
{
    std::vector<Obstacle> const obstacles = map_.confirmed();
    publish_obstacles(map_.all());

    if (!located_ || !has_goal_)
    {
        return;
    }

    std::vector<Point> const route = planner_.plan(position_, goal_, obstacles);
    if (route.empty())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "no route to (%.1f, %.1f) past %zu obstacles; holding the last plan", goal_.x, goal_.y,
                             obstacles.size());
        return;
    }

    if (!worth_publishing(route))
    {
        return;
    }

    publish_plan(route);
    published_ = route;
    RCLCPP_INFO(get_logger(), "planned %zu waypoints to (%.1f, %.1f) past %zu obstacles", route.size(), goal_.x,
                goal_.y, obstacles.size());
}

bool Planner::worth_publishing(std::vector<Point> const& route) const
{
    if (route.size() != published_.size())
    {
        return true;
    }
    for (std::size_t i = 0; i < route.size(); ++i)
    {
        if (distance(route[i], published_[i]) > replan_threshold_)
        {
            return true;
        }
    }
    return false;
}

void Planner::publish_plan(std::vector<Point> const& route) const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();

    for (Point const& waypoint : route)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = waypoint.x;
        pose.pose.position.y = waypoint.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    // Guidance reads the last pose's orientation and no other, as the heading
    // to finish on. Passing the goal's own through means an arrow dragged in
    // RViz decides which way the boat ends up pointing; a zero quaternion there
    // would ask for no particular heading.
    path.poses.back().pose.orientation = goal_orientation_;

    plan_pub_->publish(path);
}

void Planner::publish_obstacles(std::vector<Obstacle> const& obstacles) const
{
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int id = 0;
    for (Obstacle const& obstacle : obstacles)
    {
        bool const confirmed = map_.is_confirmed(obstacle);

        // Drawn at the radius the planner actually keeps clear, not the one
        // the lidar measured, so what RViz shows is what the boat will do.
        Capsule const kept_clear = inflate(obstacle.shape, inflation_);
        double const span = length(kept_clear);

        // Two namespaces rather than two colours alone, so RViz can show
        // either on its own and anything reading this can tell them apart
        // without guessing at a shade.
        visualization_msgs::msg::Marker shape;
        shape.header = clear.header;
        shape.ns = confirmed ? "confirmed" : "pending";
        shape.action = visualization_msgs::msg::Marker::ADD;
        shape.color.r = confirmed ? 0.95f : 0.45f;
        shape.color.g = confirmed ? 0.55f : 0.45f;
        shape.color.b = confirmed ? 0.15f : 0.45f;
        shape.color.a = confirmed ? 0.35f : 0.15f;
        shape.pose.orientation.w = 1.0;
        shape.scale.z = 0.2;

        // A capsule draws as its two round caps with the rectangle between
        // them. A round obstacle has no rectangle and both caps coincide, so
        // it comes out as the single cylinder it should be.
        shape.type = visualization_msgs::msg::Marker::CYLINDER;
        shape.scale.x = shape.scale.y = 2.0 * kept_clear.radius;

        shape.id = id++;
        shape.pose.position.x = kept_clear.a.x;
        shape.pose.position.y = kept_clear.a.y;
        markers.markers.push_back(shape);

        if (span > 1e-6)
        {
            shape.id = id++;
            shape.pose.position.x = kept_clear.b.x;
            shape.pose.position.y = kept_clear.b.y;
            markers.markers.push_back(shape);

            Point const centre = midpoint(kept_clear);
            visualization_msgs::msg::Marker body = shape;
            body.id = id++;
            body.type = visualization_msgs::msg::Marker::CUBE;
            body.pose.position.x = centre.x;
            body.pose.position.y = centre.y;
            body.pose.orientation =
                quaternion_of(std::atan2(kept_clear.b.y - kept_clear.a.y, kept_clear.b.x - kept_clear.a.x));
            body.scale.x = span;
            body.scale.y = 2.0 * kept_clear.radius;
            markers.markers.push_back(body);
        }

        // The hit count is what tells you whether an entry is about to be
        // confirmed or stuck one short, so it goes where it can be read.
        visualization_msgs::msg::Marker label;
        label.header = clear.header;
        label.ns = confirmed ? "confirmed_hits" : "pending_hits";
        label.id = id;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        Point const centre = midpoint(kept_clear);
        label.pose.position.x = centre.x;
        label.pose.position.y = centre.y;
        label.pose.position.z = 1.0;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.8;
        label.color.r = label.color.g = label.color.b = 1.0f;
        label.color.a = 0.9f;
        label.text = std::to_string(obstacle.hits);
        markers.markers.push_back(label);
    }

    obstacles_pub_->publish(markers);
}

}  // namespace prop_planner

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<prop_planner::Planner>());
    rclcpp::shutdown();
    return 0;
}
