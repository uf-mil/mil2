#include "detect_wall_direction.hpp"

#include <algorithm>
#include <cctype>
#include <mutex>
#include <optional>

REGISTER(DetectWallDirection)

DetectWallDirection::DetectWallDirection(std::string const& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList DetectWallDirection::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<int>("consecutive_frames", 3, "Frames the same side must persist before deciding"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    ports.insert(BT::OutputPort<std::string>("wall_side", "Decided side: left/right/front/wall"));
    return ports;
}

BT::NodeStatus DetectWallDirection::onStart()
{
    stable_side_.clear();
    stable_count_ = 0;
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
        return BT::NodeStatus::FAILURE;
    (void)getInput("consecutive_frames", need_frames_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectWallDirection::onRunning()
{
    std::optional<std::string> side_opt;
    {
        std::scoped_lock lk(ctx_->wall_direction_mx);
        side_opt = ctx_->latest_wall_direction;
    }
    if (!side_opt)
        return BT::NodeStatus::RUNNING;

    std::string side = *side_opt;
    std::transform(side.begin(), side.end(), side.begin(), [](unsigned char c) { return std::tolower(c); });

    // Ignore undecided/low-confidence frames.
    if (side != "left" && side != "right" && side != "front" && side != "wall")
    {
        stable_side_.clear();
        stable_count_ = 0;
        return BT::NodeStatus::RUNNING;
    }

    if (side == stable_side_)
        stable_count_++;
    else
    {
        stable_side_ = side;
        stable_count_ = 1;
    }

    if (stable_count_ < need_frames_)
        return BT::NodeStatus::RUNNING;

    setOutput("wall_side", side);
    RCLCPP_INFO(ctx_->logger(), "DetectWallDirection: wall=%s (stable for %d frames)", side.c_str(), stable_count_);
    return BT::NodeStatus::SUCCESS;
}

void DetectWallDirection::onHalted()
{
    if (ctx_)
        RCLCPP_WARN(ctx_->logger(), "DetectWallDirection: halted before deciding (last='%s', count=%d, need=%d)",
                    stable_side_.c_str(), stable_count_, need_frames_);
}
