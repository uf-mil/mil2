#include "hone_bearing.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yolo_msgs/msg/detection_array.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*This operation does not fail immediately if no image data has been received in an attempt to account for delays from
 * camera on startup*/

HoneBearing::HoneBearing(std::string const& name, const BT::NodeConfiguration& cfg) : BT::StatefulActionNode(name, cfg)
{
}

BT::PortsList HoneBearing::providedPorts()
{
    BT::PortsList ports;

    // Inputs
    ports.insert(BT::InputPort<std::string>("label", "shark", "Target label (single)"));
    ports.insert(BT::InputPort<double>("min_conf", 0.30, "Minimum confidence"));
    ports.insert(BT::InputPort<double>("offset_deg", 0.0, "Desired bearing offset (deg). Right positive."));
    ports.insert(BT::InputPort<double>("fov_deg", 110.0, "Horizontal FOV (deg)"));
    ports.insert(BT::InputPort<std::string>("method", "atan", "Bearing model: 'linear' or 'atan'"));
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));

    // Outputs
    ports.insert(BT::OutputPort<bool>("found"));
    ports.insert(BT::OutputPort<double>("best_cx_px"));
    ports.insert(BT::OutputPort<double>("best_conf"));

    ports.insert(BT::OutputPort<double>("bearing_deg"));
    ports.insert(BT::OutputPort<double>("error_deg"));
    ports.insert(BT::OutputPort<double>("yaw_cmd_deg"));

    return ports;
}

BT::NodeStatus HoneBearing::onStart()
{
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "HoneBearing: missing ctx");
        return BT::NodeStatus::FAILURE;
    }

    setOutput("found", false);
    setOutput("bearing_deg", 0.0);
    setOutput("error_deg", 0.0);
    setOutput("yaw_cmd_deg", 0.0);
    setOutput("best_cx_px", 0.0);
    setOutput("best_conf", 0.0);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoneBearing::onRunning()
{
    // Default inputs
    std::string label = "shark";
    std::string method = "atan";
    double min_conf = 0.30;
    double offset_deg = 0.0;
    double fov_deg = 110.0;

    (void)getInput("label", label);
    (void)getInput("method", method);
    (void)getInput("min_conf", min_conf);
    (void)getInput("offset_deg", offset_deg);
    (void)getInput("fov_deg", fov_deg);

    uint32_t W = 0;
    {
        std::scoped_lock lk(ctx_->img_mx);
        W = ctx_->img_width;
    }

    if (W == 0)
    {
        RCLCPP_WARN(ctx_->logger(), "HoneBearing: image size unknown");
        return BT::NodeStatus::RUNNING;
    }

    // Get latest detections
    std::optional<yolo_msgs::msg::DetectionArray> arr;
    {
        std::scoped_lock lk(ctx_->detections_mx);
        arr = ctx_->latest_detections;
    }

    if (!arr || arr->detections.empty())
    {
        RCLCPP_WARN(ctx_->logger(), "HoneBearing: No detections");
        setOutput("found", false);
        return BT::NodeStatus::RUNNING;
    }

    // Find best detection
    auto best = findBestDet_(*arr, label, min_conf);
    if (!best)
    {
        setOutput("found", false);
        return BT::NodeStatus::RUNNING;
    }

    // Calculate bearing
    std::optional<double> bearing_deg;
    if (method == "linear")
    {
        bearing_deg = bearingLinearDeg_(best->cx_px, W, fov_deg);
    }
    else if (method == "atan")
    {
        bearing_deg = bearingAtanDeg_(best->cx_px, W, fov_deg);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 2000,
                             "HoneBearing: unknown method '%s' (use 'linear' or 'atan')", method.c_str());
        setOutput("found", false);
        return BT::NodeStatus::FAILURE;
    }

    // In the case of mathematical errors like dividing by zero
    if (!bearing_deg)
    {
        RCLCPP_WARN(ctx_->logger(), "HoneBearing: Error Calculating Bearing");
        setOutput("found", false);
        return BT::NodeStatus::FAILURE;
    }

    // Take into account optional offset
    double const error_deg = (*bearing_deg) - offset_deg;
    double const yaw_cmd_deg = -error_deg;

    // Outputs
    setOutput("found", true);
    setOutput("best_cx_px", best->cx_px);
    setOutput("best_conf", best->conf);

    setOutput("bearing_deg", *bearing_deg);
    setOutput("error_deg", error_deg);
    setOutput("yaw_cmd_deg", yaw_cmd_deg);

    RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 400,
                         "HoneBearing[%s]: W=%u cx=%.1f conf=%.2f bearing=%.2f off=%.2f err=%.2f yaw_cmd=%.2f",
                         method.c_str(), W, best->cx_px, best->conf, *bearing_deg, offset_deg, error_deg, yaw_cmd_deg);

    return BT::NodeStatus::SUCCESS;
}

void HoneBearing::onHalted()
{
}

double HoneBearing::deg2rad(double d)
{
    return d * M_PI / 180.0;
}

double HoneBearing::rad2deg(double r)
{
    return r * 180.0 / M_PI;
}

std::optional<HoneBearing::BestDet> HoneBearing::findBestDet_(yolo_msgs::msg::DetectionArray const& arr,
                                                              std::string const& label, double min_conf)
{
    BestDet out{};
    bool found = false;

    // Get highest confidence
    for (auto const& d : arr.detections)
    {
        double const conf = d.score;

        if (d.class_name != label)
            continue;
        if (conf < min_conf)
            continue;
        if (!found || conf > out.conf)
        {
            found = true;
            out.conf = conf;
            out.cx_px = d.bbox.center.position.x;
        }
    }

    if (!found)
        return std::nullopt;

    return out;
}

std::optional<double> HoneBearing::bearingLinearDeg_(double cx_px, uint32_t width_px, double hfov_deg)
{
    if (width_px == 0)
        return std::nullopt;
    if (!(hfov_deg > 0.0 && hfov_deg < 179.0))
        return std::nullopt;

    double const W = static_cast<double>(width_px);
    double const cx0 = W * 0.5;

    // Normalized offset [-1, +1]
    double const norm = (cx_px - cx0) / cx0;

    // Linear map to [-hfov/2, +hfov/2]
    return norm * (hfov_deg * 0.5);
}

std::optional<double> HoneBearing::bearingAtanDeg_(double cx_px, uint32_t width_px, double hfov_deg)
{
    if (width_px == 0)
        return std::nullopt;
    if (!(hfov_deg > 0.0 && hfov_deg < 179.0))
        return std::nullopt;

    double const W = static_cast<double>(width_px);
    double const cx0 = W * 0.5;

    // fx = (W/2) / tan(hfov/2)
    double const hfov_rad = deg2rad(hfov_deg);
    double const half = 0.5 * hfov_rad;
    double const tan_half = std::tan(half);
    if (std::abs(tan_half) < 1e-12)
        return std::nullopt;

    double const fx = cx0 / tan_half;

    // theta = atan((x - cx)/fx)
    double const theta = std::atan((cx_px - cx0) / fx);
    return rad2deg(theta);
}
