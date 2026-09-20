#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "prop_planner/obstacle_map.hpp"
#include "prop_planner/visibility_planner.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace prop_planner
{

// Turns what the lidar has seen into a route for guidance to follow.
//
//   in   tracked_markers           visualization_msgs/MarkerArray, from pcl_tracker
//        odometry/filtered/global  nav_msgs/Odometry
//        goal_pose                 geometry_msgs/PoseStamped, RViz's "2D Goal Pose"
//   out  plan                      nav_msgs/Path, map frame, latched
//        obstacle_map              visualization_msgs/MarkerArray, what it remembers
//
// Detections are remembered in the map frame. That is the only frame here a
// moored buoy holds still in: odom is fed by an EKF with orientation and
// angular rates as its only inputs (prop_localization/config/ekf.yaml), so it
// carries the hull's rotation and none of its translation.
//
// Replanning is on a timer rather than on every detection, and a route only
// goes out when it differs from the last one. Guidance restarts its leg
// tracking on each plan it receives, so republishing the same route at
// perception rate would keep resetting the leg the boat is driving.
class Planner : public rclcpp::Node
{
  public:
    Planner();

  private:
    void tracks_callback(visualization_msgs::msg::MarkerArray const& msg);
    void replan();
    void publish_plan(std::vector<Point> const& route) const;
    void publish_obstacles(std::vector<Obstacle> const& obstacles) const;

    /// Whether the route says something new enough to interrupt guidance for.
    bool worth_publishing(std::vector<Point> const& route) const;

    std::string map_frame_;
    double replan_threshold_;  ///< m a waypoint must move before republishing
    double inflation_;         ///< kept only to draw what the planner avoids

    ObstacleMap map_;
    VisibilityPlanner planner_;

    Point position_{};
    bool located_{ false };

    Point goal_{};
    geometry_msgs::msg::Quaternion goal_orientation_;
    bool has_goal_{ false };

    std::vector<Point> published_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace prop_planner
