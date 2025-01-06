#include <mil_tools/fs/path.h>
#include <time.h>

#include <deque>
#include <filesystem>
#include <rclcpp>
#include <rosbag2_cpp/writer.hpp>
#include <unordered_map>

class OnlineBaggerServer : public rclcpp::Node
{
private:
  int successful_subscriptions;
  int total_subscriptions;
  std::unordered_map<std::string, std::vector<std::string>> groups_;
  // topic name to instances of messages received
  std::unordered_map<std::string, std::vector<rclcpp::SerializedMessage>> messages_;
  std::string bag_dir;

  // Gets todays date as YYYY-mm-dd
  std::string _today_date()
  {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
    return std::string(buf);
  }

  void _get_params()
  {
    bag_dir = this->get_parameter_or<std::string>("~bag_dir",
                                                  mil_tools::fs::path::expanduser("~/bags") + "/" + _today_date());
    if (!std::filesystem::exists(bag_dir))
    {
      std::filesystem::create_directories(bag_dir);
    }
  }

public:
  OnlineBaggerServer() : Node("online_bagger_server"), successful_subscriptions(0)
  {
    if (total_subscriptions == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "No subscriptions requested");
      rclcpp::shutdown();
    }
  }

  void add_subscription()

      void callback

      void prune()
  {
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnlineBaggerServer>());
  rclcpp::shutdown();
  return 0;
}
