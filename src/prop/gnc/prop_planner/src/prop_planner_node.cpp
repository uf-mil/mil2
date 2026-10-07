#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "prop_planner/obstacle_map.hpp"
#include "prop_planner/visibility_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
bool finite_pose(geometry_msgs::msg::Pose const &pose)
{
    auto const &p = pose.position;
    auto const &q = pose.orientation;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w);
}

double yaw_of(geometry_msgs::msg::Quaternion const &q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

prop_planner::Capsule capsule_of(geometry_msgs::msg::Pose const &pose, geometry_msgs::msg::Vector3 const &scale)
{
    double along = scale.x, across = scale.y, yaw = yaw_of(pose.orientation);
    if (across > along)
    {
        std::swap(along, across);
        yaw += 0.5 * M_PI;
    }
    double const half = 0.5 * (along - across);
    double const dx = half * std::cos(yaw), dy = half * std::sin(yaw);
    auto const &p = pose.position;
    return { { p.x - dx, p.y - dy }, { p.x + dx, p.y + dy }, across / std::sqrt(2.0) };
}
}  // namespace
class PropPlanner : public rclcpp::Node
{
  public:
    PropPlanner() : Node("prop_planner"), map_(prop_planner::ObstacleMap::Config{})
    {
        active_ = declare_parameter<bool>("active", false);
        active_callback_ = add_post_set_parameters_callback(
            [this](std::vector<rclcpp::Parameter> const &parameters)
            {
                for (auto const &parameter : parameters)
                {
                    if (parameter.get_name() != "active")
                        continue;
                    bool const active = parameter.as_bool();
                    if (active == active_)
                        continue;
                    active_ = active;
                    if (active_)
                        RCLCPP_INFO(get_logger(), "prop_planner is active now");
                }
            });
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        inflation_ = declare_parameter("inflation", 2.0);
        auto const corners = declare_parameter("corners_per_obstacle", 8);
        if (map_frame_.empty() || !std::isfinite(inflation_) || inflation_ < 0 || corners < 4 || corners > 64)
            throw std::invalid_argument("require a map_frame, finite nonnegative inflation, and 4..64 corners");
        planner_ = std::make_unique<prop_planner::VisibilityPlanner>(
            prop_planner::VisibilityPlanner::Config{ inflation_, static_cast<int>(corners) });
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        auto const odom_topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered/global");
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS(),
                                                                 [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                                 {
                                                                     if (!active_)
                                                                         return;
                                                                     if (msg->header.frame_id.empty() ||
                                                                         !finite_pose(msg->pose.pose))
                                                                         return;
                                                                     last_odom_ = *msg;
                                                                     has_odom_ = true;
                                                                 });

        auto const tracks_topic = declare_parameter<std::string>("tracks_topic", "tracked_markers");
        tracks_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            tracks_topic, rclcpp::QoS(10),
            [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) { tracks_cb(*msg); });

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", rclcpp::QoS(10),
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                if (!active_)
                    return;
                goal_ = *msg;
                has_goal_ = true;
            });
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(10));
        status_service_ = create_service<std_srvs::srv::Trigger>(
            "~/active",
            [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                response->success = active_;
                response->message = response->success ? "active" : "inactive";
            });
        toggle_service_ = create_service<std_srvs::srv::Trigger>(
            "~/toggle_active",
            [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                bool const active = !active_;
                auto const result = set_parameter(rclcpp::Parameter("active", active));
                // As with ~/active, success carries the resulting active state.
                response->success = active_;
                response->message = result.successful ? (response->success ? "active" : "inactive") : result.reason;
            });
        publish_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                           [this]()
                                           {
                                               if (!active_)
                                               {
                                                   RCLCPP_INFO_THROTTLE(get_logger(), log_clock_, 2000,
                                                                        "prop_planner is not active; toggle active to "
                                                                        "true first");
                                                   return;
                                               }
                                               path_pub_->publish(plan());
                                           });
        if (active_)
            RCLCPP_INFO(get_logger(), "prop_planner is active now");
    }

  private:
    geometry_msgs::msg::PoseStamped in_map(geometry_msgs::msg::PoseStamped pose)
    {
        if (pose.header.frame_id.empty() || !finite_pose(pose.pose))
            throw std::invalid_argument("pose requires a frame and finite coordinates/orientation");
        // Guidance uses a zero quaternion as 'no requested final heading'.
        auto const &q = pose.pose.orientation;
        double const norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        bool const no_heading = norm == 0.0;
        if (!no_heading && std::abs(norm - 1.0) > 1e-3)
            throw std::invalid_argument("orientation must be a unit quaternion or all zero");
        if (no_heading)
            pose.pose.orientation.w = 1.0;
        if (pose.header.frame_id != map_frame_)
            pose = tf_buffer_->transform(pose, map_frame_, tf2::durationFromSec(0.2));
        if (no_heading)
        {
            pose.pose.orientation.x = pose.pose.orientation.y = pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;
        }
        return pose;
    }

    void tracks_cb(visualization_msgs::msg::MarkerArray const &msg)
    {
        if (!active_)
            return;
        using Marker = visualization_msgs::msg::Marker;
        bool detection_seen = false, accepted = false;
        bool empty_scan = msg.markers.empty();
        for (auto const &marker : msg.markers)
        {
            // DELETEALL clears the current visualization, not persistent obstacle memory.
            if (marker.action == Marker::DELETEALL)
                empty_scan = true;
            if (marker.action != Marker::ADD || marker.type != Marker::CUBE)
                continue;
            detection_seen = true;
            if (!std::isfinite(marker.scale.x) || !std::isfinite(marker.scale.y) || marker.scale.x <= 0 ||
                marker.scale.y <= 0)
                continue;
            try
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = marker.header;
                pose.pose = marker.pose;
                auto const transformed = in_map(pose);
                map_.observe(capsule_of(transformed.pose, marker.scale));
                accepted = true;
            }
            catch (std::exception const &error)
            {
                RCLCPP_DEBUG(get_logger(), "Ignoring detection: %s", error.what());
            }
        }
        // An empty scan is valid input, distinct from no perception messages yet.
        if (accepted || (!detection_seen && empty_scan))
            has_map_ = true;
    }

    nav_msgs::msg::Path plan()
    {
        if (!active_)
            return nav_msgs::msg::Path{};
        auto fail = [this](char const *reason)
        {
            RCLCPP_DEBUG(get_logger(), "Planning failed: %s", reason);
            nav_msgs::msg::Path path;
            path.header.frame_id = map_frame_;
            path.header.stamp = now();
            return path;
        };
        if (!has_goal_)
            return fail("no goal received");
        if (!has_map_)
            return fail("no valid perception input received");
        if (!has_odom_)
            return fail("no odometry received");
        geometry_msgs::msg::PoseStamped start;
        start.header = last_odom_.header;
        start.pose = last_odom_.pose.pose;
        try
        {
            start = in_map(start);
            auto goal = in_map(goal_);
            prop_planner::Point const a{ start.pose.position.x, start.pose.position.y };
            prop_planner::Point const b{ goal.pose.position.x, goal.pose.position.y };
            auto const obstacles = map_.confirmed();
            // The borrowed core ignores obstacles overlapping endpoints. Preserve
            // strict endpoint clearance by rejecting those goals first.
            for (auto const &obstacle : obstacles)
            {
                auto const grown = prop_planner::inflate(obstacle.shape, inflation_);
                if (prop_planner::contains(grown, a) || prop_planner::contains(grown, b))
                    return fail("start or goal lies inside obstacle clearance");
            }
            auto const route = planner_->plan(a, b, obstacles);
            if (route.empty())
                return fail("no route exists");
            nav_msgs::msg::Path path;
            path.header.frame_id = map_frame_;
            path.header.stamp = now();
            start.header = path.header;
            path.poses.push_back(start);
            for (auto const &point : route)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = point.x;
                pose.pose.position.y = point.y;
                pose.pose.position.z = start.pose.position.z;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
            goal.header = path.header;
            path.poses.back() = goal;
            return path;
        }
        catch (std::exception const &error)
        {
            return fail(error.what());
        }
    }

    bool active_{ false };
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr active_callback_;
    std::string map_frame_;
    double inflation_;
    prop_planner::ObstacleMap map_;
    std::unique_ptr<prop_planner::VisibilityPlanner> planner_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Clock log_clock_{ RCL_STEADY_TIME };
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_service_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    geometry_msgs::msg::PoseStamped goal_;
    bool has_goal_{ false };
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_sub_;
    nav_msgs::msg::Odometry last_odom_;
    bool has_odom_{ false };
    bool has_map_{ false };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PropPlanner>());
    rclcpp::shutdown();
    return 0;
}
