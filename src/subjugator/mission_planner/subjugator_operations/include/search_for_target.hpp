#pragma once
#include <behaviortree_cpp/action_node.h>

#include <utility>
#include <vector>

#include "context.hpp"

#include <geometry_msgs/msg/pose.hpp>

// Walks an expanding spiral (search_pattern::spiral_waypoints) anchored at the
// pose where onStart runs, polling the down-cam for `label`. SUCCESS when seen;
// FAILURE when the spiral is exhausted or timeout. Step-and-wait mirrors AlignDepth.
class SearchForTarget : public BT::StatefulActionNode
{
  public:
    SearchForTarget(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    std::shared_ptr<Context> ctx_;
    std::vector<std::pair<double, double>> waypoints_;
    size_t wp_index_{ 0 };
    double center_x_{ 0.0 }, center_y_{ 0.0 }, hold_z_{ 0.0 };
    bool waiting_for_goal_{ false };
    geometry_msgs::msg::Pose pending_goal_;
    rclcpp::Time start_time_;
    bool label_seen(std::string const& label, std::string const& camera, double min_conf);
};
