#include "select_target.hpp"

#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "select_target_logic.hpp"

#include <yolo_msgs/msg/detection_array.hpp>

using select_target::Candidate;

SelectTarget::SelectTarget(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList SelectTarget::providedPorts()
{
    return { BT::InputPort<std::string>("camera", "down", "Detection stream: 'front' or 'down'"),
             BT::InputPort<double>("min_conf", 0.60, "Confidence floor (stricter than YOLO publish threshold)"),
             BT::InputPort<int>("consecutive_frames", 3, "Distinct frames the leader must win before locking"),
             BT::InputPort<std::string>("survey_labels", "nut_bolt,plug", "survey_repair target classes (csv)"),
             BT::InputPort<std::string>("rescue_labels", "pill,bandage", "search_rescue target classes (csv)"),
             BT::InputPort<std::string>("exclude", "", "Class already grabbed; skip it"),
             BT::InputPort<std::shared_ptr<Context>>("ctx"),
             BT::OutputPort<std::string>("target_label", "Chosen class for S4") };
}

BT::NodeStatus SelectTarget::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "SelectTarget: missing ctx");
        return BT::NodeStatus::FAILURE;
    }
    leader_.clear();
    votes_ = 0;
    last_stamp_ns_ = -1;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SelectTarget::onRunning()
{
    std::string camera = "down";
    std::string survey_csv = "nut_bolt,plug";
    std::string rescue_csv = "pill,bandage";
    std::string exclude;
    double min_conf = 0.60;
    int need = 3;
    (void)getInput("camera", camera);
    (void)getInput("min_conf", min_conf);
    (void)getInput("consecutive_frames", need);
    (void)getInput("survey_labels", survey_csv);
    (void)getInput("rescue_labels", rescue_csv);
    (void)getInput("exclude", exclude);

    std::string const role = ctx_->get_role();
    if (role.empty())
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "SelectTarget: role unknown; waiting");
        return BT::NodeStatus::RUNNING;
    }

    auto const targets = select_target::labels_for_role(role, survey_csv, rescue_csv);
    if (targets.empty())
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000, "SelectTarget: unrecognized role '%s'",
                             role.c_str());
        return BT::NodeStatus::RUNNING;
    }

    std::optional<yolo_msgs::msg::DetectionArray> arr = ctx_->detections_for(camera);
    if (!arr)
        return BT::NodeStatus::RUNNING;

    // Count at most one vote per distinct detection frame so a single frame
    // re-read across fast ticks can't lock the decision. stamp==0 (publisher not
    // stamping) -> treat as fresh each tick so we never stall.
    std::int64_t stamp_ns = rclcpp::Time(arr->header.stamp).nanoseconds();
    if (stamp_ns != 0 && stamp_ns <= last_stamp_ns_)
        return BT::NodeStatus::RUNNING;
    last_stamp_ns_ = stamp_ns;

    std::vector<Candidate> cands;
    cands.reserve(arr->detections.size());
    for (auto const& d : arr->detections)
        cands.push_back({ d.class_name, d.score });

    std::optional<std::string> choice = select_target::pick_best(cands, targets, min_conf, exclude);
    if (!choice)
    {
        leader_.clear();
        votes_ = 0;
        return BT::NodeStatus::RUNNING;
    }

    if (*choice == leader_)
    {
        ++votes_;
    }
    else
    {
        leader_ = *choice;
        votes_ = 1;
    }

    if (votes_ >= need)
    {
        setOutput("target_label", leader_);
        RCLCPP_INFO(ctx_->logger(), "SelectTarget: locked '%s' (%d frames)", leader_.c_str(), votes_);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}
