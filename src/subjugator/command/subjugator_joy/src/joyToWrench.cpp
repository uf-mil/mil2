#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std;

class JoySubscriber : public rclcpp::Node
{
  public:
    JoySubscriber() : Node("joy_subscriber")
    {
        publisher_1 = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_wrench", 10);
        printf("Getting joystick input \n");
        auto joy_callback = [this](sensor_msgs::msg::Joy::UniquePtr msg) -> void
        {
            auto wrench_msg = geometry_msgs::msg::Wrench();
            // the right button for directional pad gives negative values
            // x and z for force values are also flipped (negative positive)

            wrench_msg.force.x = msg->axes[1];  // move back and forth with left stick vertical
            wrench_msg.force.y = msg->axes[0];  // move right and left with left stick horizontal
            wrench_msg.force.z =
                (msg->axes[7] == 1) ? 1.0 : ((msg->axes[7] == -1.0) ? -1.0 : 0.0);  // move up and down with up and down
                                                                                    // directional pad for roll

            wrench_msg.torque.x = (msg->axes[6] == 1) ?
                                      1.0 :
                                      ((msg->axes[6] == -1.0) ? -1.0 : 0.0);  // left and right directional pad for roll
            wrench_msg.torque.y = msg->axes[4];                               // right stick vertical for pitch
            wrench_msg.torque.z = msg->axes[3];                               // right stick horizontal for pitch

            this->publisher_1->publish(wrench_msg);
        };
        subscription_1 = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_1;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}
