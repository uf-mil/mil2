#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include "context.hpp"

#include <mil_msgs/msg/perception_target_array.hpp>

class WaitForTarget : public BT::StatefulActionNode
{
  public:
    WaitForTarget(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("label"), BT::InputPort<double>("min_conf", 0.30, "Minimum confidence"),
                 BT::InputPort<int>("timeout_msec", 30000, "Timeout"), BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus onStart() override
    {
        if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "WaitForTarget: missing ctx");
            return BT::NodeStatus::FAILURE;
        }
        start_ = ctx_->node->now();
        return onRunning();
    }

    BT::NodeStatus onRunning() override
    {
        std::string label;
        double min_conf;
        int timeout_ms;
        getInput("label", label);
        getInput("min_conf", min_conf);
        getInput("timeout_msec", timeout_ms);

        auto elapsed = ctx_->node->now() - start_;
        auto timeout = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(timeout_ms) * 1000000LL);
        if (elapsed > timeout)
            return BT::NodeStatus::FAILURE;

        // check latest targets
        std::optional<mil_msgs::msg::PerceptionTargetArray> arr;
        {
            std::scoped_lock lk(ctx_->detections_mx);
            arr = ctx_->latest_targets;
        }
        if (arr)
        {
            for (auto const& t : arr->targets)
            {
                if (t.label == label && t.confidence >= min_conf)
                    return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
    }

  private:
    std::shared_ptr<Context> ctx_;
    rclcpp::Time start_;
};
