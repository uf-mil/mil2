#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>

class GPSToBaseLink : public rclcpp::Node
{
  public:
    GPSToBaseLink() : rclcpp::Node("gps_base_link_odom")
    {
        // Parameter for forward GPS antenna offset (meters)
        declare_parameter<double>("gps_offset", 0.85);
        gps_offset_ = get_parameter("gps_offset").as_double();

        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_gps", 10, std::bind(&GPSToBaseLink::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "GPSToBaseLink started with gps_offset=%.3f", gps_offset_);
    }

    ~GPSToBaseLink() override = default;  // matches rclcpp::Node

  private:
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr const msg)
    {
        // Copy input odometry
        nav_msgs::msg::Odometry new_odom = *msg;

        tf2::Quaternion quat(new_odom.pose.pose.orientation.x, new_odom.pose.pose.orientation.y,
                             new_odom.pose.pose.orientation.z, new_odom.pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        double dx = gps_offset_ * std::cos(yaw);
        double dy = gps_offset_ * std::sin(yaw);

        new_odom.pose.pose.orientation.x += dx;
        new_odom.pose.pose.orientation.y += dy;

        pub_->publish(new_odom);
    }

    double gps_offset_{ 0.85 };
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSToBaseLink>());
    rclcpp::shutdown();
    return 0;
}
