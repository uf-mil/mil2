#include "target_big_enough.hpp"

#include <algorithm>
#include <string>

#include <yolo_msgs/msg/detection_array.hpp>

REGISTER(TargetBigEnough)

BT::PortsList TargetBigEnough::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<std::string>("label", "torpedoTarget", "YOLO class label to approach"));
    ports.insert(BT::InputPort<double>("min_height_px", 200.0, "Minimum bbox height (px) to count as close enough"));
    ports.insert(BT::InputPort<double>("min_width_px", 200.0, "Minimum bbox width (px) to count as close enough"));
    ports.insert(BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"));
    ports.insert(BT::InputPort<int>("consecutive_frames", 2, "Require N consecutive good frames"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    return ports;
}

BT::NodeStatus TargetBigEnough::tick()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "TargetBigEnough: missing ctx");
        return BT::NodeStatus::FAILURE;
    }

    std::string label = "torpedoTarget";
    double min_h = 200.0, min_w = 200.0, min_conf = 0.30;
    int need_frames = 2;
    (void)getInput("label", label);
    (void)getInput("min_height_px", min_h);
    (void)getInput("min_width_px", min_w);
    (void)getInput("min_conf", min_conf);
    (void)getInput("consecutive_frames", need_frames);

    // Grab latest detections
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr || arr->detections.empty())
    {
        ok_streak_ = 0;
        RCLCPP_DEBUG_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "TargetBigEnough: no detections");
        return BT::NodeStatus::FAILURE;
    }

    // Track the largest matching detection by bbox area
    double best_w = 0.0, best_h = 0.0, best_area = 0.0;
    for (auto const& d : arr->detections)
    {
        if (d.class_name != label || d.score < min_conf)
            continue;

        double const w = std::max(0.0, d.bbox.size.x);
        double const h = std::max(0.0, d.bbox.size.y);
        double const area = w * h;
        if (area > best_area)
        {
            best_area = area;
            best_w = w;
            best_h = h;
        }
    }

    bool const ok = (best_area > 0.0) && (best_w >= min_w) && (best_h >= min_h);

    if (ok)
    {
        ok_streak_++;
        RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 500,
                             "TargetBigEnough: OK (%d/%d)  %s[h=%.0f,w=%.0f] (min h=%.0f w=%.0f)", ok_streak_,
                             need_frames, label.c_str(), best_h, best_w, min_h, min_w);

        if (ok_streak_ >= need_frames)
            return BT::NodeStatus::SUCCESS;

        return BT::NodeStatus::FAILURE;  // not enough consecutive frames yet
    }

    ok_streak_ = 0;
    RCLCPP_DEBUG_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 500,
                          "TargetBigEnough: not yet. %s[h=%.0f,w=%.0f] (min h=%.0f w=%.0f)", label.c_str(), best_h,
                          best_w, min_h, min_w);
    return BT::NodeStatus::FAILURE;
}
