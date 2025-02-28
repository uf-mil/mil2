#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"

using namespace std::chrono_literals;
using std::thread;
using std::cout;
using std::atomic;

constexpr int PUBLISH_RATE = 10; // Hertz

class SubjugatorKeyboardControl final: public rclcpp::Node
{
public:
  SubjugatorKeyboardControl()
    : Node("subjugator_keyboard_control"),
      force_x_(0.0), force_y_(0.0), force_z_(0.0),
      torque_x_(0.0), torque_y_(0.0), torque_z_(0.0),
      running_(true)
  {
    // Create publisher for Wrench messages on "cmd_wrench"
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_wrench", PUBLISH_RATE);
    this->declare_parameter("linear_speed", 0.5);
    this->declare_parameter("angular_speed", 1.0);
    base_linear_ = this->get_parameter("linear_speed").as_double();
    base_angular_ = this->get_parameter("angular_speed").as_double();

    initTerminal();

    cout << R"(Subjugator Keyboard Control:
      Up          : +x force
      Down        : -x force
      Right       : +y force
      Left        : -y force
      w           : +z force
      s           : -z force
      a           : +yaw (torque z)
      d           : -yaw (torque z)
      r           : +roll (torque x)
      f           : -roll (torque x)
      t           : +pitch (torque y)
      g           : -pitch (torque y)
      Space       : Stop all motion
      q           : Quit
    )" << '\n';

    keyboard_thread_ = thread(&SubjugatorKeyboardControl::keyboardLoop, this);
    publisher_thread_  = thread(&SubjugatorKeyboardControl::publishLoop, this);
  }

  ~SubjugatorKeyboardControl() override {
    running_ = false;
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
    if (publisher_thread_.joinable()) publisher_thread_.join();
    restoreTerminal();
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  atomic<double> force_x_, force_y_, force_z_;
  atomic<double> torque_x_, torque_y_, torque_z_;
  double base_linear_, base_angular_;
  thread keyboard_thread_;
  thread publisher_thread_;
  atomic<bool> running_;

  termios old_terminal_settings_{};
  bool terminal_initialized_{false};

  // Not exactly sure of this but ported over
  void initTerminal() {
    tcgetattr(STDIN_FILENO, &old_terminal_settings_);
    termios new_settings = old_terminal_settings_;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    terminal_initialized_ = true;
  }

  void restoreTerminal() const {
    if (terminal_initialized_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_settings_);
    }
  }

  void keyboardLoop() {
    while (running_) {
      if (const int ch = getchar(); ch == 27) { // ESC sequence for arrow keys
        if (getchar() == 91) {
          // A=up, B=down, C=right, D=left
          switch (getchar()) {
            case 'A':
              force_x_ =  base_linear_; break;
            case 'B':
              force_x_ = -base_linear_; break;
            case 'C':
              force_y_ =  base_linear_; break;
            case 'D':
              force_y_ = -base_linear_; break;
            default:
              cout << "Unknown escape char \n"; break;
          }
        }
      } else {
        switch (ch) {
          case 'w':
            force_z_ =   base_linear_; break;
          case 's':
            force_z_ =  -base_linear_; break;
          case 'a':
            torque_z_ =  base_angular_; break;
          case 'd':
            torque_z_ = -base_angular_; break;
          case 'r':
            torque_x_ =  base_angular_; break;
          case 'f':
            torque_x_ = -base_angular_; break;
          case 't':
            torque_y_ =  base_angular_; break;
          case 'g':
            torque_y_ = -base_angular_; break;
          case ' ':   // STOP
            force_x_ = force_y_ = force_z_ = 0.0;
            torque_x_ = torque_y_ = torque_z_ = 0.0;
            break;
          case 'q':    // QUIT
            running_ = false;
            rclcpp::shutdown(); // Explicit shutdown here required?
            break;
          default:
            cout << "Unknown command: " << static_cast<char>(ch) << "\n";
            break;
        }
      }
    }
  }

  void publishLoop() const {
    rclcpp::Rate rate(PUBLISH_RATE);
    geometry_msgs::msg::Wrench last_msg;
    // Initialize last_msg with zeros
    last_msg.force.x = last_msg.force.y = last_msg.force.z = 0.0;
    last_msg.torque.x = last_msg.torque.y = last_msg.torque.z = 0.0;

    while (rclcpp::ok() && running_) {
      auto current_msg = geometry_msgs::msg::Wrench();
      current_msg.force.x = force_x_.load();
      current_msg.force.y = force_y_.load();
      current_msg.force.z = force_z_.load();
      current_msg.torque.x = torque_x_.load();
      current_msg.torque.y = torque_y_.load();
      current_msg.torque.z = torque_z_.load();

      if (current_msg.force.x != last_msg.force.x ||
          current_msg.force.y != last_msg.force.y ||
          current_msg.force.z != last_msg.force.z ||
          current_msg.torque.x != last_msg.torque.x ||
          current_msg.torque.y != last_msg.torque.y ||
          current_msg.torque.z != last_msg.torque.z)
      {
        publisher_->publish(current_msg);
        last_msg = current_msg;
      }
      rate.sleep();
    }
  }
};

int main(const int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<SubjugatorKeyboardControl>();
  spin(node);
  rclcpp::shutdown();
  return 0;
}
