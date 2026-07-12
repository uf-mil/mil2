#include "target_centered.hpp"

#include <cmath>

#include <yolo_msgs/msg/detection_array.hpp>

BT::PortsList TargetCentered::providedPorts()
{
    return {
        BT::InputPort<std::string>("label", "YOLO class label to check"),
        BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
        BT::InputPort<double>("tol_norm", 0.12, "Normalised horizontal center tolerance (1.0 = image edge)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
    };
}

BT::NodeStatus TargetCentered::tick()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "TargetCentered: missing ctx");
        return BT::NodeStatus::FAILURE;
    }

    std::string label;
    double min_conf = 0.30, tol_norm = 0.12;
    (void)getInput("label", label);
    (void)getInput("min_conf", min_conf);
    (void)getInput("tol_norm", tol_norm);

    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }
    if (W == 0)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "TargetCentered: image size unknown");
        return BT::NodeStatus::FAILURE;
    }

    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }

    yolo_msgs::msg::Detection const* best = nullptr;
    double best_conf = 0.0;
    if (arr)
    {
        for (auto const& d : arr->detections)
        {
            if (d.class_name == label && d.score >= min_conf && d.score > best_conf)
            {
                best = &d;
                best_conf = d.score;
            }
        }
    }

    if (!best)
        return BT::NodeStatus::FAILURE;

    // Horizontal only: depth is set absolutely (possibly with a deliberate
    // offset), so vertical error must not block the gate.
    double const error_x =
        (best->bbox.center.position.x - static_cast<double>(W) / 2.0) / (static_cast<double>(W) / 2.0);

    bool const centered = std::abs(error_x) <= tol_norm;
    RCLCPP_DEBUG_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 500,
                          "TargetCentered[%s]: err_x=%.3f tol=%.3f -> %s", label.c_str(), error_x, tol_norm,
                          centered ? "SUCCESS" : "FAIL");
    return centered ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
