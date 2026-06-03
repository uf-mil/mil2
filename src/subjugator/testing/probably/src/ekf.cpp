#include "pch.h"

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("probably_ekf") {
        s_detections = this->create_subscription<yolo_msgs::msg::DetectionArray>
            ("/test", 10, [this](yolo_msgs::msg::DetectionArray msg) {
                detection_cb(msg);
            });
    }

    void detection_cb(yolo_msgs::msg::DetectionArray msg) {
        mrpt::obs::CActionCollection::Ptr action =
            std::make_shared<mrpt::obs::CActionCollection>();
        mrpt::obs::CSensoryFrame::Ptr sensors =
            std::make_shared<mrpt::obs::CSensoryFrame>();

        for (auto &det : msg.detections) {
            auto &center = det.bbox.center.position;
            double w2 = det.bbox.size.x / 2, h2 = det.bbox.size.y / 2;
            double left = center.x - w2, right = center.x + w2;
            double top = center.y + h2, bot = center.y - h2;
        }

        slam.processActionObservation(action, sensors);
    };

private:
    mrpt::slam::CRangeBearingKFSLAM2D slam{};
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr s_detections;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
