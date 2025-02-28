#pragma once

#include <ros2_alarms_msgs/msg/alarm.hpp>


#include <sstream>
#include <string>

using namespace ros2_alarms_msgs::msg;

namespace ros2_alarms
{
  /**
 * Representation of a ROS alarm. Used by several other classes to facilitate proper
 * interactions between components.
 */
struct AlarmProxy
{
  AlarmProxy(){};
  AlarmProxy(
    std::string alarm_name,
    bool raised, 
    std::string node_name, 
    std::string problem_description, 
    std::string json_parameters,
    int severity
  );

  AlarmProxy(
    std::string alarm_name,
    bool raised,
    std::string problem_description,
    std::string json_parameters,
    int severity
  );

  AlarmProxy(Alarm msg);

  Alarm as_msg();  // convert to ros msg

  /**
   * Prints a readable representation of the class. Normally prints the alarm name,
   * alarm status, and whether the alarm is raised or cleared. If raised, the severity
   * is also printed.
   *
   * The full representation also prints the node name, a problem description,
   * and any related JSON parameters.
   *
   * @param full Whether to print the full description.
   */
  std::string str(bool full = false) const;  // Returns printable representation of AlarmProxy

  /**
   * Overloaded equality operator. Returns true if both alarms share the same name,
   * raised status, node name, problem description, and status.
   *
   * @param other The other alarm.
   */
  bool operator==(const AlarmProxy &other) const;

  std::string alarm_name;
  bool raised = false;
  std::string node_name;
  std::string problem_description;
  std::string json_parameters;
  int severity = 0;
};

}