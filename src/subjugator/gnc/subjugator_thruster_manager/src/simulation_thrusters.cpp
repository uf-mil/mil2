#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "subjugator_msgs/msg/thruster_efforts.hpp"

class SimulatedThrusterCmds : public rclcpp::Node
{
public:
  SimulatedThrusterCmds() : Node("simulate_thruster_cmds")
  {
    publisher_flv = this->create_publisher<std_msgs::msg::Float64>("thruster/FLV", 10);
    publisher_frv = this->create_publisher<std_msgs::msg::Float64>("thruster/FRV", 10);
    publisher_blv = this->create_publisher<std_msgs::msg::Float64>("thruster/BLV", 10);
    publisher_brv = this->create_publisher<std_msgs::msg::Float64>("thruster/BRV", 10);
    auto topic_callback = [this](subjugator_msgs::msg::ThrusterEfforts::UniquePtr msg) -> void
    {
      auto msg_flv = std_msgs::msg::Float64();
      auto msg_frv = std_msgs::msg::Float64();
      auto msg_blv = std_msgs::msg::Float64();
      auto msg_brv = std_msgs::msg::Float64();
      msg_flv.data = msg->thrust_flv;
      msg_frv.data = msg->thrust_frv;
      msg_blv.data = msg->thrust_blv;
      msg_brv.data = msg->thrust_brv;
      this->publisher_flv->publish(msg_flv);
      this->publisher_frv->publish(msg_frv);
      this->publisher_blv->publish(msg_blv);
      this->publisher_brv->publish(msg_brv);
    };
    subscription_ =
        this->create_subscription<subjugator_msgs::msg::ThrusterEfforts>("thruster_efforts", 10, topic_callback);
  }

private:
  rclcpp::Subscription<subjugator_msgs::msg::ThrusterEfforts>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_flv;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_frv;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_blv;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_brv;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatedThrusterCmds>());
  rclcpp::shutdown();
  return 0;
}
