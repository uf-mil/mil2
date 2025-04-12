#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_localization/ros_filter_types.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  std::shared_ptr<robot_localization::RosEkf> filter =
    std::make_shared<robot_localization::RosEkf>(options);
  filter->initialize();
  rclcpp::spin(filter->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}