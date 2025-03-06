#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/wrench.hpp"

using namespace std::chrono_literals;
using namespace std;
//ls /opt/ros/jazzy/include/sensor_msgs/sensor_msgs/msg/

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

class JoySubscriber : public rclcpp::Node
{
public:
  JoySubscriber() : Node("joy_subscriber")
  {
    publisher_1 = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_wrench", 10);
    printf("Getting joystick input \n");
    auto joy_callback=
      [this] (sensor_msgs::msg::Joy::UniquePtr msg) -> void{
        auto wrench_msg = geometry_msgs::msg::Wrench();
          //the right button for directional pad gives negative values
          //x and z for force values are also flipped (negative positive)

          wrench_msg.force.x = msg->axes[0]; //move right and left with left stick horizontal
          wrench_msg.force.y = msg->axes[1]; //move back and forth with left stick vertical
          wrench_msg.force.z = msg->axes[4]; //move up and down with right stick vertical

          //buttons for torque, rotating around axis
          //button for positive torque and another for negative for each of them
          //using directional pad for roll (probably least used)
          
          wrench_msg.torque.x = (msg->axes[6]==1) ? 1.0 : ((msg->axes[6]==-1.0) ? -1.0: 0.0); //x and b button for roll
          wrench_msg.torque.y = msg->buttons[3] ? 1.0 : (msg->buttons[0] ? -1.0: 0.0); //y and a buttons for pitch (pointing down or up)
          wrench_msg.torque.z = msg->buttons[1] ? 1.0 : (msg->buttons[2] ? -1.0 : 0.0); // left and right of directional pad button for yaw

          //RCLCPP_INFO(this->get_logger(), "Force: [%.2f, %.2f, %.2f], Torque: [%.1f, %.1f, %.1f]",
            //wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z, wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z);
          this->publisher_1->publish(wrench_msg);
      };
    subscription_1 =
      this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_1;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoySubscriber>());
  rclcpp::shutdown();
  return 0;
}