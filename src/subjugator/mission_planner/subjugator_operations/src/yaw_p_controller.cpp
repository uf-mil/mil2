// yaw_p_controller.cpp

#include "yaw_p_controller.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

YawPController::YawPController(std::string const& name, BT::NodeConfiguration const& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList YawPController::providedPorts()
{
    BT::PortsList ports;

    // Inputs
    ports.insert(BT::InputPort<std::string>("label", "shark", "Target label (single)"));
    ports.insert(BT::InputPort<double>("min_conf", 0.30, "Minimum confidence"));
    ports.insert(BT::InputPort<double>("kp", 0.02, "P gain: yaw_cmd units per pixel error"));
    ports.insert(BT::InputPort<double>("tol_px", 8.0, "Centered tolerance in pixels"));
    ports.insert(BT::InputPort<int>("hold_ticks", 10, "Consecutive ticks within tolerance to succeed"));
    ports.insert(BT::InputPort<double>("max_cmd", 10.0, "Clamp magnitude for yaw_cmd"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));

    // Outputs
    ports.insert(BT::OutputPort<bool>("centered"));
    ports.insert(BT::OutputPort<double>("best_cx_px"));
    ports.insert(BT::OutputPort<double>("best_conf"));
    ports.insert(BT::OutputPort<double>("yaw_cmd"));

    return ports;
}

BT::NodeStatus YawPController::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "YawPController: missing ctx");
        return BT::NodeStatus::FAILURE;
    }

    stable_count_ = 0;

    setOutput("centered", false);
    setOutput("best_cx_px", 0.0);
    setOutput("best_conf", 0.0);
    setOutput("yaw_cmd", 0.0);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus YawPController::onRunning()
{
    // Read inputs
    std::string label = "shark";
    double min_conf = 0.30;
    double kp = 0.02;
    double tol_px = 8.0;
    int hold_ticks = 10;
    double max_cmd = 10.0;

    (void)getInput("label", label);
    (void)getInput("min_conf", min_conf);
    (void)getInput("kp", kp);
    (void)getInput("tol_px", tol_px);
    (void)getInput("hold_ticks", hold_ticks);
    (void)getInput("max_cmd", max_cmd);

    // Validate variables
    if (hold_ticks < 1)
    {
        RCLCPP_WARN(ctx_->logger(), "YawPController: hold_ticks must be >= 1");
        return BT::NodeStatus::FAILURE;
    }

    if (tol_px < 0.0)
    {
        RCLCPP_WARN(ctx_->logger(), "YawPController: tol_px must be >= 0");
        return BT::NodeStatus::FAILURE;
    }

    if (max_cmd < 0.0)
    {
        RCLCPP_WARN(ctx_->logger(), "YawPController: max_cmd must be >= 0");
        return BT::NodeStatus::FAILURE;
    }

    min_conf = std::clamp(min_conf, 0.0, 1.0);

    // Getting img width
    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }

    if (W == 0)
    {
        setOutput("centered", false);
        setOutput("best_cx_px", 0.0);
        setOutput("best_conf", 0.0);
        setOutput("yaw_cmd", 0.0);

        stable_count_ = 0;

        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1500, "YawPController: image size unknown");
        return BT::NodeStatus::RUNNING;
    }

    // Read latest detections
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }

    if (!arr || arr->detections.empty())
    {
        setOutput("centered", false);
        setOutput("best_cx_px", 0.0);
        setOutput("best_conf", 0.0);
        setOutput("yaw_cmd", 0.0);
        stable_count_ = 0;

        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1500, "YawPController: no detections");
        return BT::NodeStatus::RUNNING;
    }

    auto best = findBestDet_(*arr, label, min_conf);
    if (!best)
    {
        setOutput("centered", false);
        setOutput("best_cx_px", 0.0);
        setOutput("best_conf", 0.0);
        setOutput("yaw_cmd", 0.0);
        stable_count_ = 0;

        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1500,
                             "YawPController: no match for label='%s' min_conf=%.2f", label.c_str(), min_conf);
        return BT::NodeStatus::RUNNING;
    }

    // Computing pixel error relative to center
    double const cx0 = 0.5 * static_cast<double>(W);
    double const error_px = (best->cx_px - cx0);

    // P control
    double yaw_cmd = -kp * error_px;
    yaw_cmd = std::clamp(yaw_cmd, -max_cmd, +max_cmd);

    bool const centered = (std::abs(error_px) <= tol_px);
    if (centered)
        stable_count_++;
    else
        stable_count_ = 0;

    // Outputs
    setOutput("centered", centered);
    setOutput("best_cx_px", best->cx_px);
    setOutput("best_conf", best->conf);
    setOutput("yaw_cmd", yaw_cmd);

    RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 300,
                         "YawPController: W=%u cx=%.1f conf=%.2f err_px=%.1f kp=%.4f cmd=%.3f centered=%d stable=%d/%d",
                         W, best->cx_px, best->conf, error_px, kp, yaw_cmd, centered ? 1 : 0, stable_count_,
                         hold_ticks);

    // Success
    if (stable_count_ >= hold_ticks)
    {
        setOutput("yaw_cmd", 0.0);
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void YawPController::onHalted()
{
    stable_count_ = 0;
    setOutput("centered", false);
    setOutput("yaw_cmd", 0.0);
}

std::optional<YawPController::BestDet> YawPController::findBestDet_(yolo_msgs::msg::DetectionArray const& arr,
                                                                    std::string const& label, double min_conf) const
{
    BestDet out{};
    bool found = false;

    for (auto const& d : arr.detections)
    {
        double const conf = d.score;

        if (d.class_name != label)
            continue;
        if (conf < min_conf)
            continue;

        if (!found || conf > out.conf)
        {
            found = true;
            out.conf = conf;
            out.cx_px = d.bbox.center.position.x;
        }
    }

    if (!found)
        return std::nullopt;

    return out;
}
