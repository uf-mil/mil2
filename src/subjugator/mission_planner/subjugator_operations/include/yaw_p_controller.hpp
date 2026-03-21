// yaw_p_controller.hpp
#pragma once

#include <behaviortree_cpp/action_node.h>

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "context.hpp"

#include <yolo_msgs/msg/detection_array.hpp>

/**
 * YawPController
 *
 * Vision-based yaw centering using a P controller on pixel error.
 *
 * - Each tick: reads latest YOLO detections + image width, computes pixel error vs image center,
 *   outputs yaw_cmd = clamp(-kp * error_px, [-max_cmd, +max_cmd]).
 * - Returns RUNNING until centered for hold_ticks consecutive ticks, then SUCCESS.
 * - Returns FAILURE on missing ctx / overall timeout
 */
class YawPController : public BT::StatefulActionNode
{
  public:
    YawPController(std::string const& name, BT::NodeConfiguration const& cfg);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    struct BestDet
    {
        double cx_px{ 0.0 };
        double conf{ 0.0 };
    };

    std::optional<BestDet> findBestDet_(yolo_msgs::msg::DetectionArray const& arr, std::string const& label,
                                        double min_conf) const;

    static double nowSec_(rclcpp::Node const* node);

  private:
    // Shared context from input port "ctx"
    std::shared_ptr<Context> ctx_{};

    // Stateful bookkeeping across ticks
    int stable_count_{ 0 };

    // Timing (seconds)
    double start_t_{ 0.0 };
    double last_det_t_{ 0.0 };
};
