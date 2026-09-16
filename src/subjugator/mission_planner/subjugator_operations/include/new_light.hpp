#pragma once

#include <behaviortree_cpp/condition_node.h>

#include <algorithm>
#include <cmath>

#include "context.hpp"
#include "light.hpp"
#include "mil_tools/geometry/Rotation.hpp"

// SUCCESS when the front camera sees a light that no logged light explains,
// i.e. none of the same colour within `tol_deg` of its bearing from here.
// Outputs that light's colour and the yaw needed to face it.
class NewLight final : public BT::ConditionNode
{
  public:
    NewLight(std::string const& name, BT::NodeConfig const& cfg)
      : BT::ConditionNode(name, cfg), ctx_(requireContext(*this))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<Lights>("lights", "Lights logged so far"),
            BT::InputPort<double>("fov_deg", 110.0, "Front camera horizontal field of view"),
            BT::InputPort<double>("tol_deg", 15.0, "Bearings closer than this are the same light"),
            BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"),
            BT::OutputPort<std::string>("color", "Colour of the new light"),
            BT::OutputPort<double>("yaw_deg", "Relative yaw that faces the new light"),
        };
    }

  private:
    std::shared_ptr<Context> ctx_;

    BT::NodeStatus tick() override
    {
        Lights logged;
        double fov_deg = 110.0, tol_deg = 15.0;
        getInput("lights", logged);
        getInput("fov_deg", fov_deg);
        getInput("tol_deg", tol_deg);

        geometry_msgs::msg::Pose pose;
        std::optional<subjugator_msgs::msg::LightDetections> seen;
        {
            std::scoped_lock lk(ctx_->odom_mx, ctx_->lights_mx);
            pose = ctx_->latest_odom->pose.pose;
            seen = ctx_->latest_front_lights;
        }
        if (!seen)
        {
            return BT::NodeStatus::FAILURE;
        }

        double const yaw_deg = mil::geometry::Rotation{ pose.orientation }.yaw_deg();
        for (auto const& light : seen->lights)
        {
            double const to_light = -light.x * fov_deg / 2;
            double const bearing = yaw_deg + to_light;
            auto explains = [&](Light const& known)
            {
                double const to_known = std::atan2(known.y - pose.position.y, known.x - pose.position.x) * 180.0 / M_PI;
                return known.color == light.color && std::abs(std::remainder(bearing - to_known, 360.0)) < tol_deg;
            };
            if (std::none_of(logged.begin(), logged.end(), explains))
            {
                setOutput("color", light.color);
                setOutput("yaw_deg", to_light);
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }
};
