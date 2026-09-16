#pragma once

#include <behaviortree_cpp/decorator_node.h>

#include <cmath>
#include <optional>

#include "context.hpp"

// Drives its child (a body-relative SendMove) onto the `color` light one
// correction at a time: steer at it while the front camera sees it, then
// sidestep until it is centred in the down camera. Keeps creeping forward
// for `blind_steps` moves when neither camera sees it, then gives up.
// Assumes the down camera's image top is the sub's front and image right its right.
class LightFollower final : public BT::DecoratorNode
{
  public:
    LightFollower(std::string const& name, BT::NodeConfig const& cfg)
      : BT::DecoratorNode(name, cfg), ctx_(requireContext(*this))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("color", "Light to follow"),
            BT::InputPort<double>("fov_deg", 110.0, "Front camera horizontal field of view"),
            BT::InputPort<double>("step", 0.5, "Metres per correction"),
            BT::InputPort<double>("tol", 0.1, "Centred when the down-camera offset is within this (1 = edge)"),
            BT::InputPort<int>("blind_steps", 2, "Forward steps to take after losing the light"),
            BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"),
            BT::OutputPort<double>("x", "Current correction: metres forward"),
            BT::OutputPort<double>("y", "Current correction: metres left"),
            BT::OutputPort<double>("yaw_deg", "Current correction: yaw"),
        };
    }

  private:
    std::shared_ptr<Context> ctx_;
    int blind_{ 0 };

    // The `color` light nearest the image centre, if any.
    static std::optional<subjugator_msgs::msg::LightDetection>
    find(std::optional<subjugator_msgs::msg::LightDetections> const& seen, std::string const& color)
    {
        std::optional<subjugator_msgs::msg::LightDetection> best;
        if (seen)
        {
            for (auto const& light : seen->lights)
            {
                if (light.color == color && (!best || std::hypot(light.x, light.y) < std::hypot(best->x, best->y)))
                {
                    best = light;
                }
            }
        }
        return best;
    }

    void correct(double x, double y, double yaw_deg)
    {
        setOutput("x", x);
        setOutput("y", y);
        setOutput("yaw_deg", yaw_deg);
    }

    BT::NodeStatus tick() override
    {
        if (status() == BT::NodeStatus::IDLE)
        {
            blind_ = 0;
        }
        if (child_node_->status() == BT::NodeStatus::IDLE)
        {
            std::string color;
            double fov_deg = 110.0, step = 0.5, tol = 0.1;
            int blind_steps = 2;
            getInput("color", color);
            getInput("fov_deg", fov_deg);
            getInput("step", step);
            getInput("tol", tol);
            getInput("blind_steps", blind_steps);

            std::scoped_lock lk(ctx_->lights_mx);
            auto const ahead = find(ctx_->latest_front_lights, color);
            auto const below = find(ctx_->latest_down_lights, color);
            if (ahead || below)
            {
                blind_ = 0;
            }

            if (ahead)
            {
                correct(step, 0.0, -ahead->x * fov_deg / 2);
            }
            else if (below && std::abs(below->x) < tol && std::abs(below->y) < tol)
            {
                return BT::NodeStatus::SUCCESS;
            }
            else if (below)
            {
                correct(-below->y * step, -below->x * step, 0.0);
            }
            else if (blind_++ < blind_steps)
            {
                correct(step, 0.0, 0.0);
            }
            else
            {
                return BT::NodeStatus::FAILURE;
            }
        }
        auto const child_status = child_node_->executeTick();
        if (child_status != BT::NodeStatus::RUNNING)
        {
            resetChild();
        }
        return child_status == BT::NodeStatus::FAILURE ? child_status : BT::NodeStatus::RUNNING;
    }
};
