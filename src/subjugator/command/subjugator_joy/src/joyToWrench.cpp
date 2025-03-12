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
        printf("Getting joystick input, and publishing to \\cmd_wrench \n");
        this->declare_parameter("linear_speed", 100.0);   // sets the default max force outputted to the cmd_wrench
        this->declare_parameter("angular_speed", 100.0);  // sets the default max torque outputted to the cmd_wrench
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        auto joy_callback = [this](sensor_msgs::msg::Joy::UniquePtr msg) -> void
        {
            auto wrench_msg = geometry_msgs::msg::Wrench();

            wrench_msg.force.x = msg->axes[1] * linear_speed_;  // move back and forth with left stick vertical
            wrench_msg.force.y = msg->axes[0] * linear_speed_;  // move right and left with left stick horizontal
            wrench_msg.force.z = (msg->axes[7] == 1) ?
                                     linear_speed_ :
                                     ((msg->axes[7] == -1.0) ? -linear_speed_ : 0.0);  // move up and down with up and
                                                                                       // down directional pad for roll

            wrench_msg.torque.x =
                (msg->axes[6] == 1) ?
                    -angular_speed_ :
                    ((msg->axes[6] == -1.0) ? angular_speed_ : 0.0);  // left and right directional pad for roll
            wrench_msg.torque.y = msg->axes[4] * angular_speed_;      // right stick vertical for pitch
            wrench_msg.torque.z = msg->axes[3] * angular_speed_;      // right stick horizontal for pitch

            this->publisher_1->publish(wrench_msg);
        };
        subscription_1 = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_1;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_1;
    double linear_speed_;
    double angular_speed_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoySubscriber>());
    rclcpp::shutdown();
    return 0;
}
