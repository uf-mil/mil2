#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <navigator_msgs/msg/wrench_named.hpp>

namespace navigator_controller
{

class Client : public rclcpp::Node
{
  public:
    Client(std::string const& name) : rclcpp::Node(name)
    {
        wrench_pub_ = create_publisher<navigator_msgs::msg::WrenchNamed>("/wrench", 10);
    }

  protected:
    bool control_wrench(double fx, double fy, double fz, double tx, double ty, double tz)
    {
        navigator_msgs::msg::WrenchNamed msg;
        msg.source_node = get_name();
        msg.wrench.force.x = fx;
        msg.wrench.force.y = fy;
        msg.wrench.force.z = fz;
        msg.wrench.torque.x = tx;
        msg.wrench.torque.y = ty;
        msg.wrench.torque.z = tz;
        wrench_pub_->publish(msg);
        return true;
    }

  private:
    rclcpp::Publisher<navigator_msgs::msg::WrenchNamed>::SharedPtr wrench_pub_;
};

}  // namespace navigator_controller
