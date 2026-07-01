#include "select_basket.hpp"

#include <rclcpp/rclcpp.hpp>

#include "select_basket_logic.hpp"

SelectBasket::SelectBasket(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList SelectBasket::providedPorts()
{
    return { BT::InputPort<std::string>("survey_marker", "warning", "survey_repair basket marker class"),
             BT::InputPort<std::string>("rescue_marker", "red_cross", "search_rescue basket marker class"),
             BT::InputPort<std::shared_ptr<Context>>("ctx"),
             BT::OutputPort<std::string>("basket_label", "Chosen basket marker class for S5") };
}

BT::NodeStatus SelectBasket::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "SelectBasket: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SelectBasket::onRunning()
{
    std::string survey_marker = "warning";
    std::string rescue_marker = "red_cross";
    (void)getInput("survey_marker", survey_marker);
    (void)getInput("rescue_marker", rescue_marker);

    std::string const role = ctx_->get_role();
    if (role.empty())
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "SelectBasket: role unknown; waiting");
        return BT::NodeStatus::RUNNING;
    }

    std::string const marker = select_basket::marker_for_role(role, survey_marker, rescue_marker);
    if (marker.empty())
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "SelectBasket: unrecognized role '%s'",
                             role.c_str());
        return BT::NodeStatus::RUNNING;
    }

    setOutput("basket_label", marker);
    RCLCPP_INFO(ctx_->logger(), "SelectBasket: role '%s' -> basket '%s'", role.c_str(), marker.c_str());
    return BT::NodeStatus::SUCCESS;
}
