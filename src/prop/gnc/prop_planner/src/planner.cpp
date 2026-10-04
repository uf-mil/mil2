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
#include "mil_msgs/msg/perception_object.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/set_bool.hpp"
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
           declare_parameter("verify_range", 10.0), static_cast<int>(declare_parameter("max_misses", 15)),
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

    // Relative, so which estimate this follows is a remap in the launch file.
    // It has to be the one published in map_frame_: the boat's position and
    // the obstacles are compared directly, so a mismatch plans a real route
    // through the wrong water.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr const msg)
        {
            if (msg->header.frame_id != map_frame_)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                      "odometry is in %s but the map is in %s, so nothing can be planned. Remap "
                                      "odometry to the estimate published in %s, or set map_frame to %s.",
                                      msg->header.frame_id.c_str(), map_frame_.c_str(), map_frame_.c_str(),
                                      msg->header.frame_id.c_str());
                return;
            }
            position_ = { msg->pose.pose.position.x, msg->pose.pose.position.y };
            located_ = true;
        });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr const msg)
        {
            if (!enabled_)
            {
                return;
            }

            // RViz publishes a goal in whatever its Fixed Frame is set to,
            // which is not necessarily the frame being planned in. Transform
            // rather than reject, so changing that dropdown to look at
            // something does not quietly send the boat somewhere else.
            geometry_msgs::msg::PoseStamped goal;
            try
            {
                goal = tf_buffer_->transform(*msg, map_frame_, tf2::durationFromSec(0.2));
            }
            catch (tf2::TransformException const& ex)
            {
                RCLCPP_WARN(get_logger(), "goal in %s cannot be brought into %s, ignoring it: %s",
                            msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
                return;
            }

            goal_ = { goal.pose.position.x, goal.pose.position.y };
            goal_orientation_ = goal.pose.orientation;
            has_goal_ = true;
            published_.clear();  // a new goal always deserves a fresh plan
            RCLCPP_INFO(get_logger(), "new goal at (%.1f, %.1f) in %s", goal_.x, goal_.y, map_frame_.c_str());
        });

    // Guidance subscribes transient local, so a plan published before it is up
    // still reaches it.
    rclcpp::QoS latched(1);
    latched.transient_local();
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("plan", latched);
    obstacles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_map", rclcpp::QoS(1));
    objects_pub_ = create_publisher<mil_msgs::msg::PerceptionObjectArray>("obstacles", rclcpp::QoS(1));

    enabled_ = declare_parameter("start_enabled", true);
    enable_srv_ =
        create_service<std_srvs::srv::SetBool>("~/enable",
                                               [this](std_srvs::srv::SetBool::Request::SharedPtr const request,
                                                      std_srvs::srv::SetBool::Response::SharedPtr const response)
                                               {
                                                   // Enabling always starts from an empty map, so a task begins with
                                                   // what it can see rather than what the last one left behind.
                                                   if (request->data && !enabled_)
                                                   {
                                                       map_.clear();
                                                       has_goal_ = false;
                                                       published_.clear();
                                                   }
                                                   enabled_ = request->data;
                                                   response->success = true;
                                                   response->message = enabled_ ? "planning, map cleared" : "stopped";
                                                   RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
                                               });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { replan(); });

    RCLCPP_INFO(get_logger(), "planner %s - send a goal on goal_pose, or use RViz's 2D Goal Pose",
                enabled_ ? "running" : "idle, waiting on ~/enable");
}

void Planner::tracks_callback(visualization_msgs::msg::MarkerArray const& msg)
{
    if (!enabled_)
    {
        return;
    }

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

    if (located_)
    {
        map_.forget_unseen(position_);
    }
}

void Planner::replan()
{
    if (!enabled_)
    {
        return;
    }

    std::vector<Obstacle> const obstacles = map_.confirmed();
    publish_objects(map_.all());
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

namespace
{

/// Append a capsule to the array as the shapes that draw it: a cylinder at
/// each end cap, and the rectangle between them when it has any length.
///
/// Four ids are reserved per capsule so two entries can never collide on a
/// (namespace, id) pair, which is what RViz keys markers on - a collision
/// silently overwrites.
void append_capsule(visualization_msgs::msg::MarkerArray& markers, std_msgs::msg::Header const& header,
                    std::string const& ns, int base, Capsule const& capsule, std_msgs::msg::ColorRGBA const& color,
                    double height)
{
    visualization_msgs::msg::Marker cap;
    cap.header = header;
    cap.ns = ns;
    cap.action = visualization_msgs::msg::Marker::ADD;
    cap.type = visualization_msgs::msg::Marker::CYLINDER;
    cap.color = color;
    cap.pose.orientation.w = 1.0;
    cap.scale.x = cap.scale.y = 2.0 * capsule.radius;
    cap.scale.z = height;

    cap.id = base;
    cap.pose.position.x = capsule.a.x;
    cap.pose.position.y = capsule.a.y;
    markers.markers.push_back(cap);

    double const span = length(capsule);
    if (span <= 1e-6)
    {
        return;  // a round obstacle: one cap is the whole shape
    }

    cap.id = base + 1;
    cap.pose.position.x = capsule.b.x;
    cap.pose.position.y = capsule.b.y;
    markers.markers.push_back(cap);

    Point const centre = midpoint(capsule);
    visualization_msgs::msg::Marker body = cap;
    body.id = base + 2;
    body.type = visualization_msgs::msg::Marker::CUBE;
    body.pose.position.x = centre.x;
    body.pose.position.y = centre.y;
    body.pose.orientation = quaternion_of(std::atan2(capsule.b.y - capsule.a.y, capsule.b.x - capsule.a.x));
    body.scale.x = span;
    body.scale.y = 2.0 * capsule.radius;
    markers.markers.push_back(body);
}

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

}  // namespace

void Planner::publish_obstacles(std::vector<Obstacle> const& obstacles) const
{
    visualization_msgs::msg::MarkerArray markers;

    std_msgs::msg::Header header;
    header.frame_id = map_frame_;
    header.stamp = now();

    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int index = 0;
    for (Obstacle const& obstacle : obstacles)
    {
        // Eight ids per entry: four for the margin, four for the shape inside
        // it, and the label takes the last.
        int const base = 8 * index++;
        bool const confirmed = map_.is_confirmed(obstacle);
        std::string const suffix = confirmed ? "confirmed" : "pending";

        // Two rings, because they answer different questions. The outer one is
        // what the planner keeps clear and is the only one that explains the
        // route it picked; the inner one is what the lidar actually measured,
        // which at this scale the margin would otherwise swallow whole.
        append_capsule(markers, header, "margin_" + suffix, base, inflate(obstacle.shape, inflation_),
                       confirmed ? rgba(0.95f, 0.55f, 0.15f, 0.18f) : rgba(0.45f, 0.45f, 0.45f, 0.10f), 0.05);
        append_capsule(markers, header, "detected_" + suffix, base + 4, obstacle.shape,
                       confirmed ? rgba(1.0f, 0.35f, 0.05f, 0.9f) : rgba(0.6f, 0.6f, 0.6f, 0.6f), 0.6);

        // The hit count is what tells you whether an entry is about to be
        // confirmed or stuck one short, so it goes where it can be read.
        Point const centre = midpoint(obstacle.shape);
        visualization_msgs::msg::Marker label;
        label.header = header;
        label.ns = "hits_" + suffix;
        label.id = base + 7;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = centre.x;
        label.pose.position.y = centre.y;
        label.pose.position.z = 1.2;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.5;
        label.color = rgba(1.0f, 1.0f, 1.0f, 0.85f);
        label.text = std::to_string(obstacle.hits);
        markers.markers.push_back(label);
    }

    obstacles_pub_->publish(markers);
}

void Planner::publish_objects(std::vector<Obstacle> const& obstacles) const
{
    mil_msgs::msg::PerceptionObjectArray objects;
    objects.objects.reserve(obstacles.size());

    uint32_t id = 0;
    for (Obstacle const& obstacle : obstacles)
    {
        Capsule const& shape = obstacle.shape;
        Point const centre = midpoint(shape);

        mil_msgs::msg::PerceptionObject object;
        object.header.frame_id = map_frame_;
        object.header.stamp = now();
        object.id = id++;

        // The capsule, as an oriented extent: the pose is the middle of the
        // axis and the way it runs, scale.x is the axis length and scale.y the
        // full width. A round entry has a zero length axis, which is what a
        // buoy should be.
        object.pose.position.x = centre.x;
        object.pose.position.y = centre.y;
        object.pose.orientation = quaternion_of(std::atan2(shape.b.y - shape.a.y, shape.b.x - shape.a.x));
        object.scale.x = length(shape);
        object.scale.y = 2.0 * shape.radius;
        object.scale.z = 0.0;

        // Observations, and whether that is enough to be planned around.
        object.confidence = static_cast<float>(obstacle.hits);
        object.labeled_classification = map_.is_confirmed(obstacle) ? "confirmed" : "pending";

        // The axis endpoints, for anything that would rather have the capsule
        // than rebuild it from the pose.
        geometry_msgs::msg::Point32 end;
        end.x = static_cast<float>(shape.a.x);
        end.y = static_cast<float>(shape.a.y);
        object.points.push_back(end);
        end.x = static_cast<float>(shape.b.x);
        end.y = static_cast<float>(shape.b.y);
        object.points.push_back(end);

        objects.objects.push_back(object);
    }

    objects_pub_->publish(objects);
}

}  // namespace prop_planner

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<prop_planner::Planner>());
    rclcpp::shutdown();
    return 0;
}
