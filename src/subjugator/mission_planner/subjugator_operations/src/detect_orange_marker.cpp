#include "detect_orange_marker.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

BT::NodeStatus DetectOrangeMarker::tick()
{
    // Grab ctx from the blackboard
    if (!ctx_)
    {
        if (!getInput("ctx", ctx_) || !ctx_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("mission_planner"), "DetectOrangeMarker: missing ctx on blackboard");
            return BT::NodeStatus::FAILURE;
        }
    }

    double min_area_px = 1000.0;
    (void)getInput("min_area_px", min_area_px);

    // Grab the latest front cam frame
    sensor_msgs::msg::Image::SharedPtr img_msg;
    {
        std::scoped_lock lk(ctx_->front_img_mx);
        img_msg = ctx_->latest_front_image;
    }

    if (!img_msg)
    {
        RCLCPP_WARN_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 1000,
                             "DetectOrangeMarker: no front cam image yet.");
        return BT::NodeStatus::FAILURE;
    }

    // Convert ROS image to OpenCV format (same as path_marker_heading.py does)
    cv::Mat img;
    try
    {
        img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception const& e)
    {
        RCLCPP_ERROR(ctx_->logger(), "DetectOrangeMarker: cv_bridge error: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }

    // --- Same HSV pipeline as path_marker_heading.py ---
    // TODO: placeholder thresholds, replace once we have Dean's test data
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar lower_orange(5, 100, 100);
    cv::Scalar upper_orange(15, 255, 255);
    cv::Mat mask;
    cv::inRange(hsv, lower_orange, upper_orange, mask);

    // Clean up the mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::erode(mask, mask, kernel, cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 2);

    // Find contours, keep the largest one
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
        return BT::NodeStatus::FAILURE;
    }

    double best_area = 0.0;
    for (auto const& c : contours)
    {
        double area = cv::contourArea(c);
        if (area > best_area)
        {
            best_area = area;
        }
    }

    if (best_area < min_area_px)
    {
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(ctx_->logger(), "DetectOrangeMarker: found marker, area=%.0f px", best_area);
    return BT::NodeStatus::SUCCESS;
}
