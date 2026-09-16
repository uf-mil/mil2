#pragma once

#include <behaviortree_cpp/decorator_node.h>

#include <vector>

#include <Eigen/Geometry>

#include "context.hpp"
#include "mil_tools/geometry/Rotation.hpp"

// Sweeps the field ahead of the sub in a lawnmower pattern: advance `step`,
// slide to one edge, advance, slide to the other edge, until `length` is
// covered. Legs are absolute waypoints planned once from where the sub starts,
// so the child is free to wander off between them. The child must be an
// absolute SendMove reading the `x`/`y`/`yaw_deg` ports.
class LawnmowerSurvey final : public BT::DecoratorNode
{
  public:
    LawnmowerSurvey(std::string const& name, BT::NodeConfig const& cfg)
      : BT::DecoratorNode(name, cfg), ctx_(requireContext(*this))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("step", 1.0, "Forward distance between sweeps (m)"),
            BT::InputPort<double>("length", "Field length ahead of the sub (m)"),
            BT::InputPort<double>("width", "Field width, centred on the sub (m)"),
            BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"),
            BT::OutputPort<double>("x", "Current leg: odom x"),
            BT::OutputPort<double>("y", "Current leg: odom y"),
            BT::OutputPort<double>("yaw_deg", "Survey heading in odom"),
            BT::OutputPort<double>("start_x", "Where the field was planned from: odom x"),
            BT::OutputPort<double>("start_y", "Where the field was planned from: odom y"),
        };
    }

  private:
    std::shared_ptr<Context> ctx_;
    std::vector<Eigen::Vector2d> legs_;
    size_t next_{ 0 };
    double yaw_deg_{ 0.0 };

    void plan()
    {
        double step = 1.0, length = 0.0, width = 0.0;
        getInput("step", step);
        getInput("length", length);
        getInput("width", width);

        geometry_msgs::msg::Pose start;
        {
            std::scoped_lock lk(ctx_->odom_mx);
            start = ctx_->latest_odom->pose.pose;
        }
        mil::geometry::Rotation const rot{ start.orientation };
        yaw_deg_ = rot.yaw_deg();
        Eigen::Rotation2Dd const heading{ rot.yaw() };

        Eigen::Vector2d at{ start.position.x, start.position.y };
        setOutput("start_x", at.x());
        setOutput("start_y", at.y());

        double edge = width / 2, lane = 0.0;
        for (int i = length / step; i > 0; --i)
        {
            at += heading * Eigen::Vector2d{ step, 0.0 };
            legs_.push_back(at);
            at += heading * Eigen::Vector2d{ 0.0, edge - lane };
            legs_.push_back(at);
            lane = edge;
            edge = -edge;
        }
    }

    BT::NodeStatus finish(BT::NodeStatus status)
    {
        legs_.clear();
        next_ = 0;
        return status;
    }

    BT::NodeStatus tick() override
    {
        if (legs_.empty())
        {
            plan();
        }
        while (next_ < legs_.size())
        {
            if (child_node_->status() == BT::NodeStatus::IDLE)
            {
                setOutput("x", legs_[next_].x());
                setOutput("y", legs_[next_].y());
                setOutput("yaw_deg", yaw_deg_);
            }
            auto const child_status = child_node_->executeTick();
            if (child_status == BT::NodeStatus::RUNNING)
            {
                return child_status;
            }
            resetChild();
            if (child_status == BT::NodeStatus::FAILURE)
            {
                return finish(child_status);
            }
            ++next_;
        }
        return finish(BT::NodeStatus::SUCCESS);
    }
};
