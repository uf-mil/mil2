#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <navigator_msgs/msg/wrench_named.hpp>

namespace navigator_controller
{

class Server : public rclcpp::Node
{
  public:
    Server() : rclcpp::Node("navigator_controller_server")
    {
        declare_parameter("publish_frequency", 50.0);
        double freq = get_parameter("publish_frequency").as_double();

        wrench_sub_ = create_subscription<navigator_msgs::msg::WrenchNamed>(
            "/wrench", 10, [this](navigator_msgs::msg::WrenchNamed::SharedPtr msg)
            { latest_wrenches_[msg->source_node] = msg->wrench; });

        wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("/wrench_combined", 10);

        auto period = std::chrono::duration<double>(1.0 / freq);
        timer_ = create_wall_timer(period, [this]() { publish_summed_wrench(); });
    }

  private:
    void publish_summed_wrench()
    {
        geometry_msgs::msg::WrenchStamped out;
        out.header.stamp = now();
        out.header.frame_id = "base_link";

        for (auto const& [name, w] : latest_wrenches_)
        {
            out.wrench.force.x += w.force.x;
            out.wrench.force.y += w.force.y;
            out.wrench.force.z += w.force.z;
            out.wrench.torque.x += w.torque.x;
            out.wrench.torque.y += w.torque.y;
            out.wrench.torque.z += w.torque.z;
        }

        wrench_pub_->publish(out);
    }

    std::unordered_map<std::string, geometry_msgs::msg::Wrench> latest_wrenches_;
    rclcpp::Subscription<navigator_msgs::msg::WrenchNamed>::SharedPtr wrench_sub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigator_controller

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<navigator_controller::Server>());
    rclcpp::shutdown();
    return 0;
}
