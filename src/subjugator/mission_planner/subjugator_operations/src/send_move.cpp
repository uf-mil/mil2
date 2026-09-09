#include "send_move.hpp"

#include <cmath>

#include <Eigen/Dense>

#include "mil_tools/geometry/Rotation.hpp"

REGISTER(SendMove)

SendMove::SendMove(std::string const& name, BT::NodeConfiguration const& cfg) : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList SendMove::providedPorts()
{
    return {
        BT::InputPort<bool>("relative", true, "Body-relative (RelativeMove) vs odom-frame (AbsoluteMove)"),
        BT::InputPort<double>("x", 0.0, "Relative: metres forward. Absolute: odom x"),
        BT::InputPort<double>("y", 0.0, "Relative: metres left. Absolute: odom y"),
        BT::InputPort<double>("z", 0.0, "Relative: metres up. Absolute: odom z"),
        BT::InputPort<double>("roll_deg", 0.0, "Rotation about body x (deg)"),
        BT::InputPort<double>("pitch_deg", 0.0, "Rotation about body y (deg)"),
        BT::InputPort<double>("yaw_deg", 0.0, "Rotation about body z (deg)"),
        BT::InputPort<bool>("hold_x", false, "Absolute only: keep the sub's current x"),
        BT::InputPort<bool>("hold_y", false, "Absolute only: keep the sub's current y"),
        BT::InputPort<bool>("hold_z", false, "Absolute only: keep the sub's current z"),
        BT::InputPort<bool>("hold_orientation", false, "Absolute only: keep the sub's current orientation"),
        BT::InputPort<bool>("precise", false, "Tighter tolerances, longer timeout budget"),
        BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"),
    };
}

BT::NodeStatus SendMove::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "SendMove: missing ctx on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    auto num = [this](char const* port)
    {
        double v = 0.0;
        getInput(port, v);
        return v;
    };
    auto flag = [this](char const* port, bool dflt = false)
    {
        bool v = dflt;
        getInput(port, v);
        return v;
    };

    command_id_ = ctx_->nextCommandId();

    if (flag("relative", true))
    {
        subjugator_msgs::msg::RelativeMove msg;
        msg.x = num("x");
        msg.y = num("y");
        msg.z = num("z");
        msg.roll = num("roll_deg");
        msg.pitch = num("pitch_deg");
        msg.yaw = num("yaw_deg");
        msg.precise = flag("precise");
        msg.command_id = command_id_;
        ctx_->relative_move_pub->publish(msg);
    }
    else
    {
        using subjugator_msgs::msg::AbsoluteMove;
        auto hold = [&](char const* port, uint8_t bit) -> uint8_t { return flag(port) ? bit : 0; };

        AbsoluteMove msg;
        msg.target.position.x = num("x");
        msg.target.position.y = num("y");
        msg.target.position.z = num("z");
        // Same Rotation the manager uses, so both ends agree on the euler convention.
        Eigen::Vector3d const rpy_rad =
            Eigen::Vector3d{ num("roll_deg"), num("pitch_deg"), num("yaw_deg") } * (M_PI / 180.0);
        msg.target.orientation = mil::geometry::Rotation{ rpy_rad }.quat_msg();
        msg.hold = hold("hold_x", AbsoluteMove::HOLD_X) | hold("hold_y", AbsoluteMove::HOLD_Y) |
                   hold("hold_z", AbsoluteMove::HOLD_Z) | hold("hold_orientation", AbsoluteMove::HOLD_ORIENTATION);
        msg.precise = flag("precise");
        msg.command_id = command_id_;
        ctx_->absolute_move_pub->publish(msg);
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendMove::onRunning()
{
    // The move_status callback already reduced the manager's telemetry to one
    // verdict. No value yet means the move is still in flight.
    std::optional<bool> const reached = ctx_->moveVerdict(command_id_);
    if (!reached)
    {
        return BT::NodeStatus::RUNNING;
    }
    return *reached ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void SendMove::onHalted()
{
}
