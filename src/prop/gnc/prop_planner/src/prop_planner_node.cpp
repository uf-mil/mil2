#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

class PropPlanner : public rclcpp::Node
{
  public:
    PropPlanner() : Node("prop_planner")
    {
        auto const topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered/global");

        // SensorDataQoS accepts both best-effort and reliable odometry publishers.
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_cb(msg); });

        RCLCPP_INFO(get_logger(), "Waiting for odometry on %s", odom_sub_->get_topic_name());
    }

  private:
    void odom_cb(nav_msgs::msg::Odometry::ConstSharedPtr const &msg)
    {
        auto const &position = msg->pose.pose.position;
        // Use a steady clock so terminal logging is also throttled when simulation time pauses.
        RCLCPP_INFO_THROTTLE(get_logger(), log_clock_, 1000,
                             "Boat position in frame '%s': x=%.2f m, y=%.2f m, z=%.2f m",
                             msg->header.frame_id.c_str(), position.x, position.y, position.z);
    }

    rclcpp::Clock log_clock_{ RCL_STEADY_TIME };
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PropPlanner>());
    rclcpp::shutdown();
    return 0;
}
