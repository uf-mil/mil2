#include "confirm_grasp_by_scale.hpp"

#include "detection_scale.hpp"

BT::NodeStatus ConfirmGraspByScale::tick()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "ConfirmGraspByScale: missing ctx on blackboard");
        return BT::NodeStatus::FAILURE;
    }

    std::string label;
    std::string camera = "down";
    double min_conf = 0.30;
    double baseline = 0.0;
    double keep_ratio = 0.5;
    bool absent_is_grabbed = true;
    (void)getInput("label", label);
    (void)getInput("camera", camera);
    (void)getInput("min_conf", min_conf);
    (void)getInput("baseline_area", baseline);
    (void)getInput("keep_ratio", keep_ratio);
    (void)getInput("absent_is_grabbed", absent_is_grabbed);

    // No baseline (target was not seen before the lift, e.g. the down-cam model has not
    // landed): cannot verify -> stay open-loop and report success.
    if (baseline <= 0.0)
    {
        RCLCPP_WARN(ctx_->logger(), "ConfirmGraspByScale: no baseline for '%s'; assuming grasp (open-loop)",
                    label.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for(camera);
    std::optional<double> current;
    if (arr)
    {
        current = best_bbox_area(*arr, label, min_conf);
    }

    if (!current)
    {
        // Ambiguous: a held object can be occluded or too close to detect, while a missed
        // one may have drifted out of frame. Default to "grabbed" so an occluded success
        // is not retried; set absent_is_grabbed=false to be conservative instead.
        RCLCPP_WARN(ctx_->logger(), "ConfirmGraspByScale: '%s' not detected after lift; %s", label.c_str(),
                    absent_is_grabbed ? "assuming grabbed" : "assuming missed");
        return absent_is_grabbed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    double const ratio = *current / baseline;
    bool const grabbed = ratio >= keep_ratio;
    RCLCPP_INFO(ctx_->logger(), "ConfirmGraspByScale: '%s' area ratio %.2f (keep >= %.2f) -> %s", label.c_str(), ratio,
                keep_ratio, grabbed ? "GRABBED" : "MISSED");
    return grabbed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
