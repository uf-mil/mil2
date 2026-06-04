#include "nav_msgs/msg/odometry.hpp"
#include "pch.h"

class EKFNode : public rclcpp::Node
{
  public:
    EKFNode() : Node("probably_ekf")
    {
        s_odom = this->create_subscription<nav_msgs::msg::Odometry>("/dvl/", 10, [this](nav_msgs::msg::Odometry msg)
                                                                    { odom_cb(msg); });
        s_detections = this->create_subscription<yolo_msgs::msg::DetectionArray>(
            "/yolo/detections", 10, [this](yolo_msgs::msg::DetectionArray msg) { detection_cb(msg); });
    }

    void odom_cb(nav_msgs::msg::Odometry msg)
    {
        auto actions = mrpt::obs::CActionCollection::Create();
        auto sensors = mrpt::obs::CSensoryFrame::Create();

        auto range = mrpt::obs::CObservationBearingRange::Create();
        sensors->push_back(range);

        mrpt::obs::CActionRobotMovement2D action{};
        actionss.insert(action);

        slam.processActionObservation(actions, sensors);
    }

    void detection_cb(yolo_msgs::msg::DetectionArray msg)
    {
        auto actions = mrpt::obs::CActionCollection::Create();
        auto sensors = mrpt::obs::CSensoryFrame::Create();

        auto range = mrpt::obs::CObservationBearingRange::Create();
        sensors->push_back(range);

        for (auto &det : msg.detections)
        {
            auto &center = det.bbox.center.position;
            // double w2 = det.bbox.size.x / 2, h2 = det.bbox.size.y / 2;
            // double left = center.x - w2, right = center.x + w2;
            // double top = center.y + h2, bot = center.y - h2;
            // std::cout << center.x << ", " << center.y << std::endl;

            mrpt::obs::CObservationBearingRange::TMeasurement measure{};
            measure.range = 0.5;
            measure.yaw = atan2(center.x - 320, tan(40 * M_PI / 180));
            measure.landmarkID = -1;
            range->sensedData.push_back(measure);
        }

        slam.processActionObservation(actions, sensors);
    };

  private:
    mrpt::slam::CRangeBearingKFSLAM2D slam{};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr s_odom;
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr s_detections;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
