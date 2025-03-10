#include "ros2_alarms/broadcaster.hpp"

namespace ros2_alarms
{
using namespace std;

AlarmBroadcaster::AlarmBroadcaster(std::shared_ptr<rclcpp::Node> node_ptr, AlarmProxy* alarm)
  : __node_ptr(node_ptr)
  , __alarm_ptr(alarm)
  , __alarm_proxy("uninitialized_alarm", false, "", "", 5)
  , __set_alarm(__node_ptr->create_client<ros2_alarms_msgs::srv::AlarmSet>("/alarm/set"))
{
  // Broadcaster should use the AlarmProxy allocated internally if the user did
  // not provide an external one to use
  if (__alarm_ptr == nullptr)
  {
    __alarm_ptr = &__alarm_proxy;
  }
  stringstream msg;
  msg << "Node " << __alarm_proxy.node_name << " created an AlarmBroadcaster for alarm: " << __alarm_ptr->str();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", msg.str().c_str());
}

bool AlarmBroadcaster::publish()
{
  // Construct the Alarm message
  ros2_alarms_msgs::msg::Alarm a = 
      AlarmProxy(__alarm_ptr->alarm_name, __alarm_ptr->raised, __alarm_ptr->node_name, __alarm_ptr->problem_description,
                 __alarm_ptr->json_parameters, __alarm_ptr->severity)
          .as_msg();

  ros2_alarms_msgs::srv::AlarmSet::Request req;
  req.alarm = a;

  auto res = __set_alarm->async_send_request(std::make_shared<ros2_alarms_msgs::srv::AlarmSet::Request>(req));

  if (rclcpp::spin_until_future_complete(this->__node_ptr->get_node_base_interface(), res) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    if (res.get()->succeed) {  // Assuming 'success' field in the response
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Alarm successfully set.");
      return true;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to set the alarm.");
      return false;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service AlarmSet");
    return false;
  }
}

}  // namespace ros2_alarms