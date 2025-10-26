#include "check_yolo_model.hpp"

#include <chrono>
#include <filesystem>
#include <vector>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std::chrono_literals;

static bool call_transition(rclcpp::Node::SharedPtr const& node, std::string const& yolo_node, uint8_t id)
{
    auto cli = node->create_client<lifecycle_msgs::srv::ChangeState>(yolo_node + "/change_state");
    if (!cli->wait_for_service(2s))
        return false;

    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = id;

    auto fut = cli->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, fut, 3s) != rclcpp::FutureReturnCode::SUCCESS)
        return false;

    return fut.get()->success;
}

bool CheckYoloModel::ensureModelLoaded_(std::shared_ptr<Context> const& ctx, std::string const& node_name,
                                        std::string const& full_path)
{
    auto params = std::make_shared<rclcpp::AsyncParametersClient>(ctx->node, node_name);

    if (!params->wait_for_service(3s))
    {
        RCLCPP_ERROR(ctx->logger(), "CheckYoloModel: %s param service not available", node_name.c_str());
        return false;
    }

    // Read current model (if declared)
    std::string current;
    try
    {
        auto fut_get = params->get_parameters(std::vector<std::string>{ "model" });
        if (rclcpp::spin_until_future_complete(ctx->node, fut_get, 2s) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto vals = fut_get.get();  // std::vector<rclcpp::Parameter>
            if (!vals.empty() && vals.front().get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                current = vals.front().as_string();
            }
        }
    }
    catch (...)
    {
    }

    if (current == full_path)
    {
        RCLCPP_INFO(ctx->logger(), "CheckYoloModel: already using %s", full_path.c_str());
        return true;
    }

    RCLCPP_INFO(ctx->logger(), "CheckYoloModel: swapping YOLO model to %s", full_path.c_str());

    using lifecycle_msgs::msg::Transition;
    // deactivate -> cleanup
    if (!call_transition(ctx->node, node_name, Transition::TRANSITION_DEACTIVATE))
        RCLCPP_WARN(ctx->logger(), "CheckYoloModel: deactivate failed (continuing)");
    if (!call_transition(ctx->node, node_name, Transition::TRANSITION_CLEANUP))
        RCLCPP_WARN(ctx->logger(), "CheckYoloModel: cleanup failed (continuing)");

    // set parameter
    try
    {
        auto fut_set = params->set_parameters({ rclcpp::Parameter("model", full_path) });
        if (rclcpp::spin_until_future_complete(ctx->node, fut_set, 3s) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(ctx->logger(), "CheckYoloModel: set param timed out");
            return false;
        }
        auto results = fut_set.get();  // std::vector<rcl_interfaces::msg::SetParametersResult>
        if (results.empty() || !results.front().successful)
        {
            RCLCPP_ERROR(ctx->logger(), "CheckYoloModel: set param failed");
            return false;
        }
    }
    catch (std::exception const& e)
    {
        RCLCPP_ERROR(ctx->logger(), "CheckYoloModel: set param exception: %s", e.what());
        return false;
    }

    // configure -> activate
    if (!call_transition(ctx->node, node_name, Transition::TRANSITION_CONFIGURE))
    {
        RCLCPP_ERROR(ctx->logger(), "CheckYoloModel: configure failed");
        return false;
    }
    if (!call_transition(ctx->node, node_name, Transition::TRANSITION_ACTIVATE))
    {
        RCLCPP_ERROR(ctx->logger(), "CheckYoloModel: activate failed");
        return false;
    }

    // verify
    try
    {
        auto fut_verify = params->get_parameters(std::vector<std::string>{ "model" });
        if (rclcpp::spin_until_future_complete(ctx->node, fut_verify, 2s) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto vals = fut_verify.get();
            if (!vals.empty() && vals.front().get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
                vals.front().as_string() == full_path)
            {
                return true;
            }
        }
    }
    catch (...)
    {
    }

    RCLCPP_ERROR(ctx->logger(), "CheckYoloModel: verification failed");
    return false;
}

BT::NodeStatus CheckYoloModel::tick()
{
    std::shared_ptr<Context> ctx;
    if (!getInput("ctx", ctx) || !ctx)
        return BT::NodeStatus::FAILURE;

    std::string fname, yolo_node;
    std::string rel_dir = "models";  // default inside package share
    (void)getInput("model_filename", fname);
    (void)getInput("yolo_node", yolo_node);
    (void)getInput("models_rel_dir", rel_dir);

    if (fname.size() < 3 || fname.substr(fname.size() - 3) != ".pt")
        fname += ".pt";

    // Resolve via ament index: <pkg_share>/<rel_dir>/<fname>
    std::string share = ament_index_cpp::get_package_share_directory("mission_planner");
    std::filesystem::path full = std::filesystem::path(share) / rel_dir / fname;

    if (!std::filesystem::exists(full))
    {
        RCLCPP_ERROR(ctx->logger(), "CheckYoloModel: model not found: %s", full.c_str());
        return BT::NodeStatus::FAILURE;
    }

    bool ok = ensureModelLoaded_(ctx, yolo_node, full.string());
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
