#include "select_hole_depth.hpp"

#include <optional>
#include <string>

#include <yolo_msgs/msg/detection_array.hpp>

BT::PortsList SelectHoleDepth::providedPorts()
{
    return {
        BT::InputPort<std::string>("hole_label", "YOLO class of the hole we want to shoot"),
        BT::InputPort<double>("min_conf", 0.30, "Minimum detection confidence"),
        BT::InputPort<double>("z_top_large", "Absolute depth of the top task's large hole (m)"),
        BT::InputPort<double>("z_top_small", "Absolute depth of the top task's small hole (m)"),
        BT::InputPort<double>("z_bottom_large", "Absolute depth of the bottom task's large hole (m)"),
        BT::InputPort<double>("z_bottom_small", "Absolute depth of the bottom task's small hole (m)"),
        BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"),
        BT::OutputPort<double>("z", "Depth of the row carrying the requested hole"),
    };
}

BT::NodeStatus SelectHoleDepth::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SelectHoleDepth::onRunning()
{
    std::string hole_label;
    double z_top_large = 0.0, z_top_small = 0.0, z_bottom_large = 0.0, z_bottom_small = 0.0;
    if (!getInput("hole_label", hole_label) || !getInput("z_top_large", z_top_large) ||
        !getInput("z_top_small", z_top_small) || !getInput("z_bottom_large", z_bottom_large) ||
        !getInput("z_bottom_small", z_bottom_small))
    {
        RCLCPP_ERROR(ctx_->logger(), "SelectHoleDepth: hole_label and the four z_* depths are required");
        return BT::NodeStatus::FAILURE;
    }
    double min_conf = 0.15;
    (void)getInput("min_conf", min_conf);

    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }
    if (!arr)
        return BT::NodeStatus::RUNNING;

    auto is_survey = [](std::string const& label) { return label.rfind("survey&repair", 0) == 0; };
    auto is_rescue = [](std::string const& label) { return label.rfind("search&rescue", 0) == 0; };

    // Mean image-y of each task's visible holes
    double survey_y = 0.0, rescue_y = 0.0;
    int survey_n = 0, rescue_n = 0;
    for (auto const& d : arr->detections)
    {
        if (d.score < min_conf)
            continue;
        if (is_survey(d.class_name))
        {
            survey_y += d.bbox.center.position.y;
            ++survey_n;
        }
        else if (is_rescue(d.class_name))
        {
            rescue_y += d.bbox.center.position.y;
            ++rescue_n;
        }
    }

    // Need at least one hole of each task in the same frame to compare
    if (survey_n == 0 || rescue_n == 0)
        return BT::NodeStatus::RUNNING;

    // Image y grows downward: the task with the smaller mean y is on top
    bool const survey_on_top = (survey_y / survey_n) < (rescue_y / rescue_n);
    bool const hole_on_top = (is_survey(hole_label) == survey_on_top);
    bool const hole_is_large = (hole_label.find("_large") != std::string::npos);

    double const z =
        hole_on_top ? (hole_is_large ? z_top_large : z_top_small) : (hole_is_large ? z_bottom_large : z_bottom_small);
    setOutput("z", z);

    RCLCPP_INFO(ctx_->logger(), "SelectHoleDepth: %s is on top -> '%s' is the %s pair's %s hole, z=%.2f",
                survey_on_top ? "survey&repair" : "search&rescue", hole_label.c_str(), hole_on_top ? "top" : "bottom",
                hole_is_large ? "large" : "small", z);

    return BT::NodeStatus::SUCCESS;
}

void SelectHoleDepth::onHalted()
{
}
