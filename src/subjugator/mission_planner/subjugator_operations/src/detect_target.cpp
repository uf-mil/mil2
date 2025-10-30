#include "detect_target.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <vector>

#include <yolo_msgs/msg/detection_array.hpp>

BT::NodeStatus DetectTarget::tick()
{
    if (!ctx_)
    {
        if (!getInput("ctx", ctx_) || !ctx_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "DetectTarget: missing ctx on blackboard");
            return BT::NodeStatus::FAILURE;
        }
    }

    std::string label;
    double min_conf = 0.40;
    (void)getInput("label", label);
    (void)getInput("min_conf", min_conf);

    if (label.empty())
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "DetectTarget: empty label.");
        return BT::NodeStatus::FAILURE;
    }

    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr)
        return BT::NodeStatus::FAILURE;

    for (auto const& det : arr->detections)
        if (det.class_name == label && det.score >= min_conf)
            return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::FAILURE;
}
