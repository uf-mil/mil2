#include "common.hpp"

class EKFNode : public rclcpp::Node
{
  public:
    EKFNode() : Node("probably_ekf")
    {
        declare_parameter("chi2_thres", 0.95);

        // slam.options.data_assoc_method = mrpt::slam::assocJCBB;
        // slam.options.data_assoc_IC_chi2_thres = get_parameter("chi2_thres").as_double();
        s_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, [this](nav_msgs::msg::Odometry msg)
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

        mrpt::obs::CActionRobotMovement2D action{};
        action.timestamp = mrpt::Clock::now();
        auto pose = mrpt::poses::CPosePDFGaussian::Create();
        pose->mean.x(odom_diff.x);
        pose->mean.y(odom_diff.y);

        pose->mean.phi(odom_diff.phi);
        pose->cov(0, 0) = odom_diff.cov[0];
        pose->cov(1, 1) = odom_diff.cov[4];
        pose->cov(2, 2) = odom_diff.cov[8];
        action.poseChange = pose;

        // auto pose = mrpt::poses::CPose2D{};
        // pose.x(odom_diff.x);
        // pose.y(odom_diff.x);
        // pose.phi(odom_diff.phi);
        // action->computeFromOdometry(pose, mrpt::obs::CActionRobotMovement2D::TMotionModelOptions{});

        actions->insert(action);

        auto range = mrpt::obs::CObservationBearingRange::Create();
        range->validCovariances = false;
        range->timestamp = mrpt::Clock::now();

        // std::cout << "----------" << msg.detections.size() << std::endl;
        for (auto &det : msg.detections)
        {
            auto &center = det.bbox.center.position;
            double w2 = det.bbox.size.x / 2, h2 = det.bbox.size.y / 2;
            // double left = center.x - w2, right = center.x + w2;
            // double top = center.y + h2, bot = center.y - h2;
            double diag = sqrt(4 * (w2 * w2 + h2 * h2));
            // std::cout << center.x << ", " << center.y << std::endl;

            mrpt::obs::CObservationBearingRange::TMeasurement measure{};
            measure.range = (320 / diag) * (0.9144 / tan(40 * M_PI / 180));
            measure.yaw = atan2(center.x - 320, 320 / tan(40 * M_PI / 180));
            // measure.covariance(0, 0) = 5; // range
            // measure.covariance(1, 1) = 1; // yaw
            measure.landmarkID = 1; // -1;
            range->sensedData.push_back(measure);
            // std::cout << measure.yaw << ", " << measure.range << std::endl;;
        }

        sensors->push_back(range);

        slam.processActionObservation(actions, sensors);
        log();
    }

    void log() {
        mrpt::poses::CPosePDFGaussian pose;
        std::vector<mrpt::math::TPoint2D> landmarkPositions;
        std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> landmarkIDs;
        mrpt::math::CVectorDouble fullState;
        mrpt::math::CMatrixDouble fullCovariance;

        slam.getCurrentState(pose,
                             landmarkPositions,
                             landmarkIDs,
                             fullState,
                             fullCovariance);

        std::cout << pose.mean.x() << ", " << pose.mean.y() << "," << pose.mean.phi() << std::endl;

        std::cout << "----------" << landmarkPositions.size() << std::endl;
        for (int i = 0; i < landmarkPositions.size(); ++i) {
            std::cout << landmarkIDs[i] << ": " << landmarkPositions[i] << std::endl;
        }
    }

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
