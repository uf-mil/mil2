#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <cmath>

#include "context.hpp"

#include <mil_msgs/msg/perception_target_array.hpp>
#include <sensor_msgs/msg/image.hpp>

class AlignBearing : public BT::StatefulActionNode
{
  public:
    AlignBearing(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("label"),
                 BT::InputPort<double>("desired_bearing_deg", 30.0, "Desired pole bearing (+right of center)"),
                 BT::InputPort<double>("tolerance_deg", 5.0, "Acceptable error"),
                 BT::InputPort<double>("fov_deg", 90.0, "Horizontal FOV"),
                 BT::InputPort<double>("max_step_deg", 12.0, "Max yaw per command"),
                 BT::InputPort<double>("min_conf", 0.30, "Min confidence"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus onStart() override
    {
        if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "AlignBearing: missing ctx");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        std::string label;
        double desired, tol, fov, max_step, min_conf;
        getInput("label", label);
        getInput("desired_bearing_deg", desired);
        getInput("tolerance_deg", tol);
        getInput("fov_deg", fov);
        getInput("max_step_deg", max_step);
        getInput("min_conf", min_conf);

        // need image width
        uint32_t W = 0, H = 0;
        {
            std::scoped_lock lk(ctx_->img_mx);
            W = ctx_->img_width;
            H = ctx_->img_height;
        }
        if (W == 0)
        {
            RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "AlignBearing: no image width yet.");
            return BT::NodeStatus::RUNNING;
        }

        // find best matching detection
        std::optional<mil_msgs::msg::PerceptionTargetArray> arr;
        {
            std::scoped_lock lk(ctx_->detections_mx);
            arr = ctx_->latest_targets;
        }
        if (!arr || arr->targets.empty())
            return BT::NodeStatus::RUNNING;

        mil_msgs::msg::PerceptionTarget const* best = nullptr;
        float best_conf = 0.f;
        for (auto const& t : arr->targets)
            if (t.label == label && t.confidence >= min_conf && t.confidence > best_conf)
            {
                best = &t;
                best_conf = t.confidence;
            }

        if (!best)
            return BT::NodeStatus::RUNNING;

        // pixel -> bearing (deg). Center=0, right positive.
        double cx = best->cx;
        double bearing = ((cx - (double)W / 2.0) / ((double)W / 2.0)) * (fov / 2.0);

        double err = desired - bearing;  // we want bearing -> desired
        if (std::fabs(err) <= tol)
            return BT::NodeStatus::SUCCESS;

        // choose yaw delta (left positive). After a left yaw of +d, bearing becomes (bearing - d).
        // Want: bearing - d = desired  => d = bearing - desired = -err
        double yaw_delta = std::clamp(bearing - desired, -max_step, max_step);

        // Publish a relative yaw-only goal
        geometry_msgs::msg::Pose current{};
        {
            std::scoped_lock lk(ctx_->odom_mx);
            if (ctx_->latest_odom)
                current = ctx_->latest_odom->pose.pose;
        }

        geometry_msgs::msg::Pose goal;
        goal.position = current.position;
        double rad = yaw_delta * M_PI / 180.0;
        goal.orientation.x = 0.0;
        goal.orientation.y = 0.0;
        goal.orientation.z = std::sin(rad / 2.0);
        goal.orientation.w = std::cos(rad / 2.0);

        // Compose relative: q_new = q_cur * q_delta (just reuse PublishGoalPose semantics)
        geometry_msgs::msg::Pose out = current;
        auto const& c = current.orientation;
        auto const& d = goal.orientation;
        out.orientation.x = c.w * d.x + c.x * d.w + c.y * d.z - c.z * d.y;
        out.orientation.y = c.w * d.y - c.x * d.z + c.y * d.w + c.z * d.x;
        out.orientation.z = c.w * d.z + c.x * d.y - c.y * d.x + c.z * d.w;
        out.orientation.w = c.w * d.w - c.x * d.x - c.y * d.y - c.z * d.z;

        ctx_->goal_pub->publish(out);
        RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 500,
                             "AlignBearing: bearing=%.1f°, desired=%.1f°, yaw_delta=%.1f°", bearing, desired,
                             yaw_delta);

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
    }

  private:
    std::shared_ptr<Context> ctx_;
};
