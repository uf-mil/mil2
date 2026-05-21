#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <optional>
#include <string>

#include "context.hpp"
#include "operations.hpp"

#include <geometry_msgs/msg/pose.hpp>

// Captures the most recently published goal pose (ctx->last_goal) and stores
// it in ctx->waypoints under the given name. Using last_goal (instead of odom)
// means we remember the commanded point, not where the sub physically settled.
// FAILURE if no goal has been published yet.
class RememberWaypoint final : public BT::SyncActionNode, public OperationBase
{
  public:
    RememberWaypoint(std::string const& name, BT::NodeConfiguration const& cfg)
      : BT::SyncActionNode(name, cfg), OperationBase(cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("name", "Name to store the last goal pose under"),
            BT::InputPort<bool>("overwrite", true, "Overwrite existing waypoint with same name"),
            BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"),
        };
    }

    BT::NodeStatus tick() override
    {
        if (!ctx_)
        {
            if (!getInput("ctx", ctx_) || !ctx_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "RememberWaypoint: missing ctx on blackboard");
                return BT::NodeStatus::FAILURE;
            }
        }

        std::string wp_name;
        if (!getInput("name", wp_name) || wp_name.empty())
        {
            RCLCPP_ERROR(ctx_->logger(), "RememberWaypoint: missing or empty 'name' input");
            return BT::NodeStatus::FAILURE;
        }

        bool overwrite = true;
        (void)getInput("overwrite", overwrite);

        std::optional<geometry_msgs::msg::Pose> last_goal;
        {
            std::scoped_lock lk(ctx_->last_goal_mx);
            last_goal = ctx_->last_goal;
        }
        if (!last_goal)
        {
            RCLCPP_ERROR(ctx_->logger(),
                         "RememberWaypoint '%s': no last_goal yet; publish a goal before "
                         "remembering",
                         wp_name.c_str());
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Pose const pose = *last_goal;
        {
            std::scoped_lock lk(ctx_->waypoints_mx);
            auto it = ctx_->waypoints.find(wp_name);
            if (it != ctx_->waypoints.end() && !overwrite)
            {
                RCLCPP_WARN(ctx_->logger(), "RememberWaypoint '%s' already exists and overwrite=false; skipping",
                            wp_name.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            ctx_->waypoints[wp_name] = pose;
        }

        RCLCPP_INFO(ctx_->logger(),
                    "RememberWaypoint '%s' captured last_goal pos(%.2f,%.2f,%.2f) quat(%.3f,%.3f,%.3f,%.3f)",
                    wp_name.c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                    pose.orientation.y, pose.orientation.z, pose.orientation.w);

        return BT::NodeStatus::SUCCESS;
    }
};
