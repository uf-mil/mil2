#pragma once

#include "rclcpp/rclcpp.hpp"
#include "ros2_alarms_msgs/msg/alarm.hpp"
#include "ros2_alarms_msgs/srv/alarm_set.hpp"

#include <std_msgs/msg/header.hpp>
#include <functional>
#include <ros2_alarms/alarm_proxy.hpp>
#include <string>

namespace ros2_alarms
{
template <typename msg_t = std_msgs::msg::Header>
class HeartbeatMonitor: public rclcpp::Node
{
public:

  HeartbeatMonitor(
      std::shared_ptr<rclcpp::Node> node_ptr, std::string alarm_name, std::string heartbeat_topic,
      std::function<bool(msg_t)> predicate = [](msg_t) { return true; },
      rclcpp::Duration time_to_raise = rclcpp::Duration(0.1), rclcpp::Duration time_to_clear = rclcpp::Duration(0.1),
      rclcpp::Duration period = rclcpp::Duration(0.02));
  // TODO: find out why a period < 0.02 causes callbacks not to be called

  std::string alarm_name() const;

  std::string heartbeat_name() const;

  bool healthy() const;

  rclcpp::Time getLastBeatTime() const;

  int getNumConnections() const;

  bool waitForConnection(rclcpp::Duration timeout = { -1.0 }) const;  // waits forever by default

  void startMonitoring();

  void stopMonitoring();

private:
  rclcpp::Subscription<msg_t>::SharedPtr __heartbeat_listener;
  rclcpp::TimerBase::SharedPtr __status_checker;
  rclcpp::Duration __time_to_raise;
  rclcpp::Duration __time_to_clear;
  rclcpp::Duration __period;
  rclcpp::Time __last_beat;
  rclcpp::Duration __time_recovering;
  bool __recovering = false;
  bool __healthy = true;
  std::string __object_name;

  AlarmProxy __alarm_proxy;
  AlarmBroadcaster __alarm_broadcaster;
  AlarmListener<> __alarm_listener;

  // Callback for when a heartbeat message is received
  void __heardHeartbeat(const msg_t::SharedPtr beat_msg);

  // Callback for checking for heartbeat loss
  void __checkForHeartbeatLoss();
};
}  // namespace ros_alarms