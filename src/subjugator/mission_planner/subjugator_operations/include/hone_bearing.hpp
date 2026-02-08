#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <optional>
#include <string>

#include "context.hpp"

#include <yolo_msgs/msg/detection_array.hpp>

class HoneBearing : public BT::StatefulActionNode
{
  public:
    HoneBearing(std::string const& name, const BT::NodeConfiguration& cfg);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    static double deg2rad(double deg);
    static double rad2deg(double rad);

    struct BestDet
    {
        double cx_px{ 0.0 };
        double conf{ 0.0 };
    };

    static std::optional<BestDet> findBestDet_(yolo_msgs::msg::DetectionArray const& arr, std::string const& label,
                                               double min_conf);

    static std::optional<double> bearingLinearDeg_(double cx_px, uint32_t width_px, double hfov_deg);
    static std::optional<double> bearingAtanDeg_(double cx_px, uint32_t width_px, double hfov_deg);

    std::shared_ptr<Context> ctx_;
};
