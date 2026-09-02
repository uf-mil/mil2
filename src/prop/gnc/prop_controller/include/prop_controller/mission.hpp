#pragma once

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"

// Publishes a mission file as the path for guidance to follow.
//
//   out  plan   nav_msgs/Path, map frame, latched
//
// Missions are flat lists of map frame x, y waypoints in
// share/prop_controller/missions/<name>.yaml, selected with the "mission"
// launch argument. A perception driven planner would publish this same topic
// instead.
class Mission : public rclcpp::Node
{
  public:
    Mission();

  private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
};
