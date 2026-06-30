#include "select_face_symbol.hpp"

#include <rclcpp/rclcpp.hpp>

#include "select_face_symbol_logic.hpp"

BT::NodeStatus SelectFaceSymbol::tick()
{
    std::shared_ptr<Context> ctx;
    if (!getInput("ctx", ctx) || !ctx)
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "SelectFaceSymbol: missing ctx");
        return BT::NodeStatus::FAILURE;
    }

    std::string role;
    int items = 0;
    (void)getInput("role", role);
    (void)getInput("items_collected", items);

    if (role.empty())
        role = ctx->get_role();

    std::string const symbol = select_face_symbol::symbol_for(role, items);
    setOutput("target_symbol", symbol);

    RCLCPP_INFO(ctx->logger(), "SelectFaceSymbol: role='%s' items=%d -> symbol='%s'", role.c_str(), items,
                symbol.c_str());
    return BT::NodeStatus::SUCCESS;
}
