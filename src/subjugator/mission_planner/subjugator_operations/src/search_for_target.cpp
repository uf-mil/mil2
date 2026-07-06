#include "search_for_target.hpp"

#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "detection_gate.hpp"
#include "search_pattern.hpp"

SearchForTarget::SearchForTarget(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList SearchForTarget::providedPorts()
{
    return {
        BT::InputPort<std::string>("label", "table", "YOLO class label to find"),
        BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
        BT::InputPort<double>("min_conf", 0.40, "Minimum detection confidence"),
        BT::InputPort<double>("step_m", 0.30, "Spiral spacing (m)"),
        BT::InputPort<double>("max_radius_m", 2.0, "Search radius cap (m)"),
        BT::InputPort<double>("pos_tol", 0.15, "Goal-reached tolerance (m)"),
        BT::InputPort<int>("timeout_msec", 60000, "Overall timeout (ms)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

bool SearchForTarget::label_seen(std::string const& label, std::string const& camera, double min_conf)
{
    auto arr = ctx_->detections_for(camera);
    return arr && detection_gate::contains_label(*arr, label, min_conf);
}

BT::NodeStatus SearchForTarget::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "SearchForTarget: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    double step_m = 0.30, max_radius = 2.0;
    getInput("step_m", step_m);
    getInput("max_radius_m", max_radius);

    geometry_msgs::msg::Pose cur{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            cur = ctx_->latest_odom->pose.pose;
    }
    center_x_ = cur.position.x;
    center_y_ = cur.position.y;
    hold_z_ = cur.position.z;

    waypoints_ = search_pattern::spiral_waypoints(step_m, max_radius);
    wp_index_ = 0;
    waiting_for_goal_ = false;
    start_time_ = ctx_->node->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchForTarget::onRunning()
{
    std::string label = "table", camera = "down";
    double min_conf = 0.40, pos_tol = 0.15;
    int timeout_msec = 60000;
    getInput("label", label);
    getInput("camera", camera);
    getInput("min_conf", min_conf);
    getInput("pos_tol", pos_tol);
    getInput("timeout_msec", timeout_msec);

    if (label_seen(label, camera, min_conf))
    {
        RCLCPP_INFO(ctx_->logger(), "SearchForTarget: '%s' found at waypoint %zu", label.c_str(), wp_index_);
        return BT::NodeStatus::SUCCESS;
    }

    if ((ctx_->node->now() - start_time_).seconds() * 1000.0 > timeout_msec)
    {
        RCLCPP_WARN(ctx_->logger(), "SearchForTarget: timed out without '%s'", label.c_str());
        return BT::NodeStatus::FAILURE;
    }

    if (waiting_for_goal_)
    {
        geometry_msgs::msg::Pose cur{};
        {
            std::scoped_lock lk(ctx_->odom_mx);
            if (ctx_->latest_odom)
                cur = ctx_->latest_odom->pose.pose;
        }
        double dx = cur.position.x - pending_goal_.position.x;
        double dy = cur.position.y - pending_goal_.position.y;
        if (std::hypot(dx, dy) > pos_tol)
            return BT::NodeStatus::RUNNING;
        waiting_for_goal_ = false;
    }

    if (wp_index_ >= waypoints_.size())
    {
        RCLCPP_WARN(ctx_->logger(), "SearchForTarget: exhausted %zu waypoints, '%s' not found", waypoints_.size(),
                    label.c_str());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Pose goal{};
    {
        std::scoped_lock lk(ctx_->odom_mx);
        if (ctx_->latest_odom)
            goal = ctx_->latest_odom->pose.pose;  // keep current orientation
    }
    goal.position.x = center_x_ + waypoints_[wp_index_].first;
    goal.position.y = center_y_ + waypoints_[wp_index_].second;
    goal.position.z = hold_z_;
    ctx_->goal_pub->publish(goal);
    {
        std::scoped_lock lk(ctx_->last_goal_mx);
        ctx_->last_goal = goal;
    }
    pending_goal_ = goal;
    waiting_for_goal_ = true;
    ++wp_index_;
    return BT::NodeStatus::RUNNING;
}

void SearchForTarget::onHalted()
{
    waiting_for_goal_ = false;
}
