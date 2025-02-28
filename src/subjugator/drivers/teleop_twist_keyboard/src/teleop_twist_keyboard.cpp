#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::thread;
using std::cout;
using std::atomic;

const int PUBLISH_RATE = 10; // Hertz

class TeleopTwistKeyboard : public rclcpp::Node
{
public:
  TeleopTwistKeyboard() : Node("teleop_twist_keyboard"), linear_(0.0), angular_(0.0), running_(true)
  {
    // Create publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", PUBLISH_RATE);
    this->declare_parameter("linear_speed", 0.5), this->declare_parameter("angular_speed", 1.0);
    base_linear_ = this->get_parameter("linear_speed").as_double();
    base_angular_ = this->get_parameter("angular_speed").as_double();

    initTerminal();

    std::map<std::string, std::string> commands = {
      {"i", "forward"},
      {",", "backward"},
      {"j", "turn left"},
      {"l", "turn right"},
      {"k", "stop"},
      {"q", "quit"}
    };

    cout << "Teleop Keyboard:\n";  // Output commands
    for (const auto& [k, v] : commands) cout<<"  "<<k<<": "<<v<<'\n';

    keyboard_thread_ = thread(&TeleopTwistKeyboard::keyboardLoop, this);
    publisher_thread_ = thread(&TeleopTwistKeyboard::publishLoop, this);
  }

  ~TeleopTwistKeyboard() override {
    running_ = false;
    if (keyboard_thread_.joinable())  keyboard_thread_.join();
    if (publisher_thread_.joinable()) publisher_thread_.join();
    restoreTerminal();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  atomic<double> linear_, angular_;      // Current speed balues
  double base_linear_, base_angular_;  // THese control
  thread keyboard_thread_;
  thread publisher_thread_;
  atomic<bool> running_;

  // For terminal settings
  struct termios old_terminal_settings_;
  bool terminal_initialized_{false};

  // Set the terminal in raw mode (non-canonical, no echo), and reset
  void initTerminal() {
    tcgetattr(STDIN_FILENO, &old_terminal_settings_);
    struct termios new_settings = old_terminal_settings_;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    terminal_initialized_ = true;
  }
  void restoreTerminal() {
    if (terminal_initialized_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_settings_);
    }
  }

// Input
  // TOLOOK: If performance is not super critical, can consolidate this into a map
  void keyboardLoop() {
    while (running_) {
      int ch = getchar();
      switch(ch) {
        case 'i': linear_ =  base_linear_; angular_ = 0.0;            break;
        case ',': linear_ = -base_linear_; angular_ = 0.0;            break;
        case 'j': linear_ = 0.0;           angular_ = base_angular_;  break;
        case 'l': linear_ = 0.0;           angular_ = -base_angular_; break;
        case 'k': linear_ = 0.0;           angular_ = 0.0;            break;
        case 'q':
          running_ = false; break;
        default:
          cout<<"HUH keyboardLoop\n";
          break;
      }
    }
  }

// Output
  void publishLoop() {
    rclcpp::Rate rate(PUBLISH_RATE);
    while (rclcpp::ok() && running_) {
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = linear_.load(), twist_msg.angular.z = angular_.load();
      publisher_->publish(twist_msg);
      rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopTwistKeyboard>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
