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
class HeartbeatMonitor
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
  std::shared_ptr<rclcpp::Node> __node_ptr;
  std::string __heartbeat_topic;
  AlarmProxy __alarm_proxy;
  std::function<bool(msg_t)> __msg_predicate;
  AlarmBroadcaster __alarm_broadcaster;
  AlarmListener<> __alarm_listener;
//   ros::CallbackQueue __cb_queue;
//   ros::AsyncSpinner __async_spinner;
//   ros::Subscriber __heartbeat_listener;
//   ros::Timer __status_checker;
//   ros::Duration __time_to_raise;
//   ros::Duration __time_to_clear;
//   ros::Duration __period;
//   ros::Time __last_beat = { 0, 0 };
//   ros::Duration __time_recovering = { 0, 0 };
//   bool __recovering = false;
//   bool __healthy = true;
//   std::string __object_name;
//   void __heardHeartbeat(msg_t beat_msg);                    // Callback for heartbeat topic subscriber
//   void __checkForHeartbeatLoss(const ros::TimerEvent &te);  // Callback for __status_checker
};
}