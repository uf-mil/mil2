#include "prop_planner/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
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

}  // namespace

Planner::Planner()
  : Node("planner")
  , map_({ declare_parameter("merge_distance", 2.0), declare_parameter("position_gain", 0.2),
           static_cast<int>(declare_parameter("min_hits", 3)), declare_parameter("max_radius", 5.0),
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

        geometry_msgs::msg::Point position;
        tf2::doTransform(marker.pose.position, position, transform);

        // The tracker's boxes are axis aligned in the sensor's frame and carry
        // no orientation of their own, so their footprint only means anything
        // as a circle. The circumscribing one is the honest reading of it.
        map_.observe(position.x, position.y, 0.5 * std::hypot(marker.scale.x, marker.scale.y));
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

        visualization_msgs::msg::Marker disc;
        disc.header = clear.header;
        // Two namespaces rather than two colours alone, so RViz can show either
        // on its own and anything reading this can tell them apart without
        // guessing at a shade.
        disc.ns = confirmed ? "confirmed" : "pending";
        disc.id = id++;
        disc.type = visualization_msgs::msg::Marker::CYLINDER;
        disc.action = visualization_msgs::msg::Marker::ADD;
        disc.pose.position.x = obstacle.x;
        disc.pose.position.y = obstacle.y;
        disc.pose.orientation.w = 1.0;

        // Drawn at the radius the planner actually keeps clear, not the one the
        // lidar measured, so what RViz shows is what the boat will do.
        double const kept_clear = obstacle.radius + inflation_;
        disc.scale.x = disc.scale.y = 2.0 * kept_clear;
        disc.scale.z = 0.2;
        disc.color.r = confirmed ? 0.95f : 0.45f;
        disc.color.g = confirmed ? 0.55f : 0.45f;
        disc.color.b = confirmed ? 0.15f : 0.45f;
        disc.color.a = confirmed ? 0.35f : 0.15f;
        markers.markers.push_back(disc);

        // The hit count is what tells you whether an entry is about to be
        // confirmed or stuck one short, so it goes where it can be read.
        visualization_msgs::msg::Marker label;
        label.header = clear.header;
        label.ns = confirmed ? "confirmed_hits" : "pending_hits";
        label.id = disc.id;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = obstacle.x;
        label.pose.position.y = obstacle.y;
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
