#include "poles_big_enough.hpp"

#include <algorithm>

#include <yolo_msgs/msg/detection_array.hpp>

BT::PortsList PolesBigEnough::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("min_height_px", 120.0, "Minimum bbox height (px)"));
    ports.insert(BT::InputPort<double>("min_width_px", 12.0, "Minimum bbox width (px)"));
    ports.insert(BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"));
    ports.insert(BT::InputPort<int>("consecutive_frames", 2, "Require N consecutive good frames"));
    ports.insert(BT::InputPort<bool>("need_both", true, "Require both red and white poles"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    return ports;
}

BT::NodeStatus PolesBigEnough::tick()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "PolesBigEnough: missing ctx");
        return BT::NodeStatus::FAILURE;
    }

    double min_h = 120.0, min_w = 12.0, min_conf = 0.30;
    int need_frames = 2;
    bool need_both = true;
    (void)getInput("min_height_px", min_h);
    (void)getInput("min_width_px", min_w);
    (void)getInput("min_conf", min_conf);
    (void)getInput("consecutive_frames", need_frames);
    (void)getInput("need_both", need_both);

    // Grab latest detections
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr || arr->detections.empty())
    {
        ok_streak_ = 0;
        RCLCPP_DEBUG_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "PolesBigEnough: no detections");
        return BT::NodeStatus::FAILURE;
    }

    // Track the largest red and white poles by area
    double best_red_w = 0.0, best_red_h = 0.0, best_red_area = 0.0;
    double best_white_w = 0.0, best_white_h = 0.0, best_white_area = 0.0;

    for (auto const& d : arr->detections)
    {
        if (d.score < min_conf)
            continue;

        double const w = std::max(0.0, d.bbox.size.x);
        double const h = std::max(0.0, d.bbox.size.y);
        double const area = w * h;

        if (d.class_name == "red-pole" && area > best_red_area)
        {
            best_red_area = area;
            best_red_w = w;
            best_red_h = h;
        }
        else if (d.class_name == "white-pole" && area > best_white_area)
        {
            best_white_area = area;
            best_white_w = w;
            best_white_h = h;
        }
    }

    bool const red_ok = (best_red_w >= min_w) && (best_red_h >= min_h);
    bool const white_ok = (best_white_w >= min_w) && (best_white_h >= min_h);
    bool const ok = need_both ? (red_ok && white_ok) : (red_ok || white_ok);

    if (ok)
    {
        ok_streak_++;
        RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 500,
                             "PolesBigEnough: OK (%d/%d)  red[h=%.0f,w=%.0f] white[h=%.0f,w=%.0f] need_both=%d",
                             ok_streak_, need_frames, best_red_h, best_red_w, best_white_h, best_white_w,
                             (int)need_both);

        if (ok_streak_ >= need_frames)
            return BT::NodeStatus::SUCCESS;

        return BT::NodeStatus::FAILURE;  // not enough consecutive frames yet
    }
    else
    {
        ok_streak_ = 0;
        RCLCPP_DEBUG_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 500,
                              "PolesBigEnough: not yet. red[h=%.0f,w=%.0f] white[h=%.0f,w=%.0f] need_both=%d (min "
                              "h=%.0f w=%.0f)",
                              best_red_h, best_red_w, best_white_h, best_white_w, (int)need_both, min_h, min_w);
        return BT::NodeStatus::FAILURE;
    }
}
