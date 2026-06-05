#include "common.hpp"

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

    void odom_cb(nav_msgs::msg::Odometry &msg)
    {
        if (!has_odom) {
            odom_prev = msg;
            has_odom = true;
        }
        odom_last = msg;
    }

    void detection_cb(yolo_msgs::msg::DetectionArray &msg)
    {
        if (!has_odom) return;

        auto odom_next = extrapolate_odom(odom_last);
        auto odom_diff = subtract_odom(odom_prev, odom_next);
        odom_prev = odom_last;

        auto actions = mrpt::obs::CActionCollection::Create();
        auto sensors = mrpt::obs::CSensoryFrame::Create();

        auto action = mrpt::obs::CActionRobotMovement2D::Create();
        actions->insertPtr(action);
        auto pose = mrpt::poses::CPosePDFGaussian::Create();
        pose->mean.x(odom_diff.x);
        pose->mean.y(odom_diff.x);
        pose->mean.phi(odom_diff.phi);
        action->poseChange = pose;

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

    bool has_odom = false;
    nav_msgs::msg::Odometry odom_prev{};
    nav_msgs::msg::Odometry odom_last{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
