#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

#include <mutex>

#include "context.hpp"
#include "geometry_msgs/msg/pose.hpp"

class YawStyle : public BT::SyncActionNode
{
  public:
    YawStyle(std::string const& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
    {
        get_port_data();
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<Context>>("ctx"),
        };
    }

  private:
    std::shared_ptr<Context> ctx_;

    void get_port_data()
    {
        auto ctx_res = getInput<std::shared_ptr<Context>>("ctx");
        if (!ctx_res)
        {
            throw BT::RuntimeError("TopicTicker requires [ctx] input");
        }
        ctx_ = ctx_res.value();
    }

  protected:
    BT::NodeStatus tick() override
    {
        // read values from ctx, if no value then fail
        geometry_msgs::msg::Pose last_goal;
        {
            std::scoped_lock lk(ctx_->last_goal_mx);
            // we must have moved at least once b4 running style points ig
            if (!ctx_->last_goal.has_value())
            {
                ctx_->last_goal.emplace(geometry_msgs::msg::Pose());
                // return BT::NodeStatus::FAILURE;
            }
            last_goal = *ctx_->last_goal;
        }

        // use that value (plus a 90 degree rotation)
        double const MAGIC_NUMBER = 0.70710678118;  // root 2 / 2 for rotation by 90 degrees

        double x1 = 0;
        double y1 = 0;
        double z1 = MAGIC_NUMBER;
        double w1 = MAGIC_NUMBER;

        double x2 = last_goal.orientation.x;
        double y2 = last_goal.orientation.y;
        double z2 = last_goal.orientation.z;
        double w2 = last_goal.orientation.w;

        // evil math since eigen didn't want to work
        double x3 = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        double y3 = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        double z3 = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
        double w3 = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;

        last_goal.orientation.x = x3;
        last_goal.orientation.y = y3;
        last_goal.orientation.z = z3;
        last_goal.orientation.w = w3;

        ctx_->goal_pub->publish(last_goal);
        {
            std::scoped_lock lk(ctx_->last_goal_mx);
            ctx_->last_goal.emplace(last_goal);
        }

        return BT::NodeStatus::SUCCESS;
    }
};
