#include "record_target_scale.hpp"

#include "detection_scale.hpp"

BT::NodeStatus RecordTargetScale::tick()
{
    if (!require_ctx(*this, ctx_, "RecordTargetScale"))
    {
        return BT::NodeStatus::FAILURE;
    }

    std::string label;
    std::string camera = "down";
    double min_conf = 0.30;
    (void)getInput("label", label);
    (void)getInput("camera", camera);
    (void)getInput("min_conf", min_conf);

    double baseline = 0.0;
    if (!label.empty())
    {
        std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for(camera);
        if (arr)
        {
            if (auto area = best_bbox_area(*arr, label, min_conf))
            {
                baseline = *area;
            }
        }
    }

    if (baseline <= 0.0)
    {
        RCLCPP_WARN(ctx_->logger(), "RecordTargetScale: no '%s' detection on %s cam; grasp will not be scale-verified",
                    label.c_str(), camera.c_str());
    }
    else
    {
        RCLCPP_INFO(ctx_->logger(), "RecordTargetScale: baseline area for '%s' = %.0f px^2", label.c_str(), baseline);
    }

    setOutput("baseline_area", baseline);
    return BT::NodeStatus::SUCCESS;
}
