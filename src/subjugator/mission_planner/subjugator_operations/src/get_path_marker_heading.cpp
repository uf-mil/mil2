#include "get_path_marker_heading.hpp"

BT::NodeStatus GetPathMarkerHeading::onStart()
{
    // Grab ctx from the blackboard
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "GetPathMarkerHeading: missing ctx");
        return BT::NodeStatus::FAILURE;
    }

    // Read timeout parameter
    (void)getInput("timeout_sec", timeout_sec_);

    // Clear any stale heading from a previous run so we don't
    // accidentally reuse it
    {
        std::scoped_lock lk(ctx_->path_marker_mx);
        ctx_->latest_path_marker_heading.reset();
    }

    // Record when we started so we can time out
    start_time_ = ctx_->node->get_clock()->now();

    RCLCPP_INFO(ctx_->logger(),
                "GetPathMarkerHeading: waiting for /path_marker/heading "
                "(timeout %.1f s)",
                timeout_sec_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetPathMarkerHeading::onRunning()
{
    // Check if path_marker_heading.py has published anything yet
    std::optional<geometry_msgs::msg::QuaternionStamped> msg;
    {
        std::scoped_lock lk(ctx_->path_marker_mx);
        msg = ctx_->latest_path_marker_heading;
    }

    if (msg.has_value())
    {
        // We got a heading -- write its quaternion to the output ports
        // so the XML tree can pass them to PublishGoalPose
        auto const& q = msg->quaternion;
        setOutput("qx", q.x);
        setOutput("qy", q.y);
        setOutput("qz", q.z);
        setOutput("qw", q.w);

        RCLCPP_INFO(ctx_->logger(),
                    "GetPathMarkerHeading: got heading "
                    "qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
                    q.x, q.y, q.z, q.w);

        return BT::NodeStatus::SUCCESS;
    }

    // No heading yet -- check if we've timed out
    double elapsed = (ctx_->node->get_clock()->now() - start_time_).seconds();

    if (elapsed > timeout_sec_)
    {
        RCLCPP_ERROR(ctx_->logger(),
                     "GetPathMarkerHeading: timed out after %.1f s "
                     "with no heading received",
                     elapsed);
        return BT::NodeStatus::FAILURE;
    }

    // Still waiting
    RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 2000,
                         "GetPathMarkerHeading: still waiting... (%.1f s)", elapsed);

    return BT::NodeStatus::RUNNING;
}
