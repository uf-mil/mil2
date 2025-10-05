#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <cmath>

#include "context.hpp"

#include <mil_msgs/msg/perception_target_array.hpp>

class AdvanceUntilLost : public BT::StatefulActionNode
{
  public:
    AdvanceUntilLost(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("label"),
                 BT::InputPort<double>("step_m", 0.35, "Forward step (m)"),
                 BT::InputPort<int>("interval_msec", 800, "Publish interval"),
                 BT::InputPort<int>("lose_count", 5, "Consecutive frames w/o target to declare lost"),
                 BT::InputPort<double>("min_conf", 0.30, "Min confidence"),
                 BT::InputPort<int>("max_steps", 50, "Safety bound"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus onStart() override
    {
        if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "AdvanceUntilLost: missing ctx");
            return BT::NodeStatus::FAILURE;
        }
        not_seen_ = 0;
        steps_ = 0;
        last_pub_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        std::string label;
        double step;
        int interval_ms, lose_count, max_steps;
        double min_conf;
        getInput("label", label);
        getInput("step_m", step);
        getInput("interval_msec", interval_ms);
        getInput("lose_count", lose_count);
        getInput("max_steps", max_steps);
        getInput("min_conf", min_conf);

        bool seen = false;
        {  // check detection
            std::optional<mil_msgs::msg::PerceptionTargetArray> arr;
            std::scoped_lock lk(ctx_->detections_mx);
            arr = ctx_->latest_targets;
            if (arr)
                for (auto const& t : arr->targets)
                    if (t.label == label && t.confidence >= min_conf)
                    {
                        seen = true;
                        break;
                    }
        }

        if (seen)
            not_seen_ = 0;
        else
            not_seen_++;

        if (not_seen_ >= lose_count)
            return BT::NodeStatus::SUCCESS;

        // publish forward step periodically
        auto now = ctx_->node->now();

        auto since = now - last_pub_;
        auto interval = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(interval_ms) * 1000000LL);

        if (since >= interval && steps_ < max_steps)
        {
            geometry_msgs::msg::Pose current{};
            {
                std::scoped_lock lk(ctx_->odom_mx);
                if (ctx_->latest_odom)
                    current = ctx_->latest_odom->pose.pose;
            }

            // forward vector in body frame (x=+step, y=z=0) rotated by current yaw
            auto const& q = current.orientation;
            double vx = step, vy = 0, vz = 0;
            double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
            double w1 = -qx * vx - qy * vy - qz * vz;
            double x1 = qw * vx + qy * vz - qz * vy;
            double y1 = qw * vy + qz * vx - qx * vz;
            double z1 = qw * vz + qx * vy - qy * vx;

            geometry_msgs::msg::Pose goal = current;
            goal.position.x = current.position.x + (x1 * qw - w1 * qx - y1 * qz + z1 * qy);
            goal.position.y = current.position.y + (y1 * qw - w1 * qy - z1 * qx + x1 * qz);
            goal.position.z = current.position.z + (z1 * qw - w1 * qz - x1 * qy + y1 * qx);
            goal.orientation = current.orientation;  // keep heading

            ctx_->goal_pub->publish(goal);
            last_pub_ = now;
            steps_++;
            RCLCPP_INFO(ctx_->logger(), "AdvanceUntilLost: step %d, forward %.2fm", steps_, step);
        }

        if (steps_ >= max_steps)
            return BT::NodeStatus::SUCCESS;

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
    }

  private:
    std::shared_ptr<Context> ctx_;
    rclcpp::Time last_pub_;
    int not_seen_{ 0 };
    int steps_{ 0 };
};
