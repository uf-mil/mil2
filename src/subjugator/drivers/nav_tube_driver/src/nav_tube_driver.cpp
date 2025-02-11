#include <boost/asio.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mil_msgs/msg/depth_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

using tcp = boost::asio::ip::tcp;

class NavTubeDriver : public rclcpp::Node
{
private:
  rclcpp::Publisher<mil_msgs::msg::DepthStamped>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry recent_odom_msg_;

  std::string ip_;
  int port_;
  std::string frame_id_;
  uint16_t hz_;

  uint64_t acceptable_frequency;

  rclcpp::Time prev;

  std::thread timer_thread;

  std::mutex m;

  bool running = true;

  bool initialized = false;

  static uint8_t const sync1 = 0x37;
  static uint8_t const sync2 = 0x01;

  uint8_t heartbeat_packet[2 + sizeof(hz_)];

  boost::shared_ptr<tcp::socket> connect();

  void send_heartbeat(boost::shared_ptr<tcp::socket> socket);

  void read_messages(boost::shared_ptr<tcp::socket> socket);

  double calculate_pressure(uint16_t analog_input);

public:
  NavTubeDriver();

  ~NavTubeDriver();

  void run();
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr const msg);
};

NavTubeDriver::NavTubeDriver() : rclcpp::Node("nav_tube_driver")
{
  pub_ = this->create_publisher<mil_msgs::msg::DepthStamped>("depth", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&NavTubeDriver::odom_callback, this, std::placeholders::_1));

  ip_ = this->declare_parameter<std::string>("ip", "192.168.37.61");
  port_ = this->declare_parameter<int>("port", 33056);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "/depth");

  int hz__ = this->declare_parameter<int>("hz", 20);

  if (hz__ > std::numeric_limits<uint16_t>::max())
  {
    RCLCPP_WARN(this->get_logger(), "Depth polling frequency is greater than 16 bits!");
  }

  hz_ = hz__;

  acceptable_frequency = (1'000'000'000 * 1.25) / (uint64_t)hz_;

  heartbeat_packet[0] = sync1;
  heartbeat_packet[1] = sync2;
  uint16_t nw_ordering = htons(hz_);
  reinterpret_cast<uint16_t *>(&heartbeat_packet[2])[0] = nw_ordering;
}

NavTubeDriver::~NavTubeDriver()
{
  {
    std::lock_guard<std::mutex> lock(m);
    running = false;
  }
  timer_thread.join();
}

void NavTubeDriver::odom_callback(nav_msgs::msg::Odometry::SharedPtr const ptr)
{
  recent_odom_msg_ = *ptr;
}

boost::shared_ptr<tcp::socket> NavTubeDriver::connect()
{
  using ip_address = boost::asio::ip::address;
  tcp::endpoint endpoint(ip_address::from_string(ip_), port_);

  RCLCPP_INFO(this->get_logger(), "Connecting to Depth Server");
  boost::asio::io_service io_service;
  boost::shared_ptr<tcp::socket> socket = boost::make_shared<tcp::socket>(io_service);
  socket->connect(endpoint);
  RCLCPP_INFO(this->get_logger(), "Connection to Depth Server established");

  return socket;
}

void NavTubeDriver::send_heartbeat(boost::shared_ptr<tcp::socket> socket)
{
  try
  {
    while (rclcpp::ok())
    {
      boost::asio::write(*socket, boost::asio::buffer(heartbeat_packet));

      {
        std::lock_guard<std::mutex> lock(m);
        if (!running)
        {
          return;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  catch (boost::system::system_error const &e)
  {
  }
}

void NavTubeDriver::read_messages(boost::shared_ptr<tcp::socket> socket)
{
  mil_msgs::msg::DepthStamped msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = rclcpp::Clock().now();

  uint8_t backing[10];

  auto buffer = boost::asio::buffer(backing, sizeof(backing));

  while (rclcpp::ok())
  {
    if (rclcpp::Clock().now().nanoseconds() - prev.nanoseconds() > static_cast<long>(acceptable_frequency))
    {
      RCLCPP_WARN(this->get_logger(), "Depth sampling rate is falling behind.");
    }

    if (!boost::asio::buffer_size(buffer))
    {
      // Bytes are out of sync so try and resync
      if (backing[0] != sync1 || backing[1] != sync2)
      {
        for (int i = 0; i < int(sizeof(backing) / sizeof(backing[0])) - 1; i++)
        {
          backing[i] = backing[i + 1];
        }
        buffer = boost::asio::buffer(backing + (sizeof(backing) / sizeof(backing[0])) - sizeof(backing[0]),
                                     sizeof(backing[0]));
      }
      else
      {
        msg.header.stamp = rclcpp::Clock().now();

        uint64_t bits = be64toh(*reinterpret_cast<uint64_t *>(&backing[2]));
        double pressure = *reinterpret_cast<double *>(&bits);
        if (recent_odom_msg_.header.stamp.sec > msg.header.stamp.sec)
        {
          // Accounts for the dynamic pressure applied to the pressure sensor
          // when the sub is moving forwards or backwards
          double velocity = recent_odom_msg_.twist.twist.linear.x;
          double vel_effect = (abs(velocity) * velocity) / (1000 * 9.81);
          msg.depth = pressure + vel_effect;
        }
        else
        {
          msg.depth = pressure;
        }

        pub_->publish(msg);
        buffer = boost::asio::buffer(backing, sizeof(backing));
      }
    }

    size_t bytes_read = socket->read_some(buffer);

    buffer = boost::asio::buffer(buffer + bytes_read);
    prev = rclcpp::Clock().now();

    rclcpp::spin_some(this->get_node_base_interface());
  }
}

void NavTubeDriver::run()
{
  while (rclcpp::ok())
  {
    try
    {
      prev = rclcpp::Clock().now();
      boost::shared_ptr<tcp::socket> socket;

      socket = connect();
      timer_thread = std::thread(&NavTubeDriver::send_heartbeat, this, socket);
      initialized = true;
      read_messages(socket);
    }
    catch (boost::system::system_error const &e)
    {
      std::chrono::seconds wait_time(5);
      RCLCPP_WARN(this->get_logger(), "Error with NavTube Depth driver TCP socket %s. Trying again in %ld seconds",
                  e.what(), wait_time.count());

      if (initialized)
      {
        timer_thread.join();
      }
      initialized = false;
      std::this_thread::sleep_for(wait_time);
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NavTubeDriver>();
  node->run();

  rclcpp::shutdown();
  return 0;
}
