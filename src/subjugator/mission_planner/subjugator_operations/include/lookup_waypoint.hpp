#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>

#include "context.hpp"
#include "operations.hpp"

#include <geometry_msgs/msg/pose.hpp>

// Looks up a named waypoint in ctx->waypoints and writes its absolute pose
// fields to output ports for downstream nodes (e.g. PublishGoalPose, AtGoalPose).
// FAILURE if the name is not in the store.
class LookupWaypoint final : public BT::SyncActionNode, public OperationBase
{
  public:
    LookupWaypoint(std::string const& name, BT::NodeConfiguration const& cfg)
      : BT::SyncActionNode(name, cfg), OperationBase(cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("name", "Waypoint name to look up"),
            BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"),
            BT::OutputPort<double>("x"),
            BT::OutputPort<double>("y"),
            BT::OutputPort<double>("z"),
            BT::OutputPort<double>("qx"),
            BT::OutputPort<double>("qy"),
            BT::OutputPort<double>("qz"),
            BT::OutputPort<double>("qw"),
        };
    }

    BT::NodeStatus tick() override
    {
        if (!ctx_)
        {
            if (!getInput("ctx", ctx_) || !ctx_)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "LookupWaypoint: missing ctx on blackboard");
                return BT::NodeStatus::FAILURE;
            }
        }

        std::string wp_name;
        if (!getInput("name", wp_name) || wp_name.empty())
        {
            RCLCPP_ERROR(ctx_->logger(), "LookupWaypoint: missing or empty 'name' input");
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Pose pose;
        {
            std::scoped_lock lk(ctx_->waypoints_mx);
            auto it = ctx_->waypoints.find(wp_name);
            if (it == ctx_->waypoints.end())
            {
                RCLCPP_ERROR(ctx_->logger(), "LookupWaypoint: name '%s' not in waypoint store", wp_name.c_str());
                return BT::NodeStatus::FAILURE;
            }
            pose = it->second;
        }

        setOutput("x", pose.position.x);
        setOutput("y", pose.position.y);
        setOutput("z", pose.position.z);
        setOutput("qx", pose.orientation.x);
        setOutput("qy", pose.orientation.y);
        setOutput("qz", pose.orientation.z);
        setOutput("qw", pose.orientation.w);

        RCLCPP_INFO(ctx_->logger(), "LookupWaypoint '%s' -> pos(%.2f,%.2f,%.2f) quat(%.3f,%.3f,%.3f,%.3f)",
                    wp_name.c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                    pose.orientation.y, pose.orientation.z, pose.orientation.w);

        return BT::NodeStatus::SUCCESS;
    }
};
