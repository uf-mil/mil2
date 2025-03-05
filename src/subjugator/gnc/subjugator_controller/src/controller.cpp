#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
// // #include <control_toolbox/pid.hpp>

class PIDController : public rclcpp::Node
{
  public:
    //   subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, odom_callback);
    // private:
    //   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    //   // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_trajectory_;
}

// void odom_callback(nav_msgs::msg::Odometry::UniquePtr msg) {
//   // std::cout << msg << std::endl;
//   // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
// }

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}
