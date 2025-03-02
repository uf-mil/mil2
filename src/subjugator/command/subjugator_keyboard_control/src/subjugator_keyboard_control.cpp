#include <chrono>
#include <iostream>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using std::thread;
using std::cout;
using std::atomic;

constexpr int PUBLISH_RATE = 10; // Hertz

// May be used elsewhere if similar logic?
struct KeyState {
  bool pressed = false;
  steady_clock::time_point last_time = steady_clock::now();
};
struct AxisEffect {
  KeyState* key;
  double multiplier;
  double* target; // What we're updating
};

class SubjugatorKeyboardControl final: public rclcpp::Node {
public:
  SubjugatorKeyboardControl()
    : Node("subjugator_keyboard_control"),
      force_x_(0.0), force_y_(0.0), force_z_(0.0),
      torque_x_(0.0), torque_y_(0.0), torque_z_(0.0),
      running_(true)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_wrench", PUBLISH_RATE);
    this->declare_parameter("linear_speed", 0.5);
    this->declare_parameter("angular_speed", 1.0);
    base_linear_ = this->get_parameter("linear_speed").as_double();
    base_angular_ = this->get_parameter("angular_speed").as_double();

    initTerminal();

    // Start keyboard and publisher threads
    keyboard_thread_  = thread(&SubjugatorKeyboardControl::keyboardLoop, this);
    publisher_thread_ = thread(&SubjugatorKeyboardControl::publishLoop, this);

    cout << R"(Subjugator Keyboard Control:
      Up          : +x force
      Down        : -x force
      Right       : +y force
      Left        : -y force
      w           : +z force
      s           : -z force
      a           : -yaw (torque z)
      d           : +yaw (torque z)
      r           : +roll (torque x)
      f           : -roll (torque x)
      t           : +pitch (torque y)
      g           : -pitch (torque y)
      Space       : Stop all motion
      q           : Quit
    )" << '\n';
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

  void initTerminal() {
    tcgetattr(STDIN_FILENO, &old_terminal_settings_);
    termios new_settings = old_terminal_settings_;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    new_settings.c_cc[VMIN] = 0;  // Set non-blocking input
    new_settings.c_cc[VTIME] = 1; // 0.1 second timeout
    // These flags are necessary so doesn't always return -1
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    terminal_initialized_ = true;
  }

  void restoreTerminal() const {
    if (terminal_initialized_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_settings_);
    }
  }

  void keyboardLoop() {
    KeyState arrow_up, arrow_down, arrow_left, arrow_right;
    KeyState key_w, key_s, key_a, key_d, key_r, key_f, key_t, key_g;

    double current_force_x = 0.0,  current_force_y = 0.0,  current_force_z = 0.0;
    double current_torque_x = 0.0, current_torque_y = 0.0, current_torque_z = 0.0;

    std::vector<KeyState*> keys = {
      &arrow_up, &arrow_down, &arrow_left, &arrow_right,
      &key_w, &key_s, &key_a, &key_d,
      &key_r, &key_f, &key_t, &key_g
    };

    AxisEffect letter_effects[] = {
      { &arrow_up,     base_linear_, &current_force_x },
      { &arrow_down,  -base_linear_, &current_force_x },
      { &arrow_right,  base_linear_, &current_force_y },
      { &arrow_left,  -base_linear_, &current_force_y },
      { &key_w,  base_linear_,  &current_force_z },
      { &key_s, -base_linear_,  &current_force_z },
      { &key_d,  base_angular_, &current_torque_z },
      { &key_a, -base_angular_, &current_torque_z },
      { &key_r,  base_angular_, &current_torque_x },
      { &key_f, -base_angular_, &current_torque_x },
      { &key_t,  base_angular_, &current_torque_y },
      { &key_g, -base_angular_, &current_torque_y }
    };
    const auto timeout = milliseconds(150);

    while (running_) {
      auto now = steady_clock::now();

      // Update key timeouts first?
      for (auto key : keys) {
        if (now - key->last_time > timeout) key->pressed = false;
      }

      // Reset every time
      current_force_x = 0.0,  current_force_y = 0.0,  current_force_z = 0.0;
      current_torque_x = 0.0, current_torque_y = 0.0, current_torque_z = 0.0;

      int ch = getchar();
      if (ch == -1) {
        // Do nothing. Avoids additional nesting or use of goto
      } else if (ch == 27) {  // Possible arrow key escape sequence
        int ch2 = getchar();
        if (ch2 == 91) {
          int ch3 = getchar();
          if (ch3 == 'A') { arrow_up.pressed = true; arrow_up.last_time = now; }
          else if (ch3 == 'B') { arrow_down.pressed = true; arrow_down.last_time = now; }
          else if (ch3 == 'C') { arrow_right.pressed = true; arrow_right.last_time = now; }
          else if (ch3 == 'D') { arrow_left.pressed = true; arrow_left.last_time = now; }
          else {
            cout << "Unknown escape sequence\n";
          }
        }
      } else {
        // Process normal keys:
        switch(ch) {
          case 'w': key_w.pressed = true; key_w.last_time = now; break;
          case 's': key_s.pressed = true; key_s.last_time = now; break;
          case 'a': key_a.pressed = true; key_a.last_time = now; break;
          case 'd': key_d.pressed = true; key_d.last_time = now; break;
          case 'r': key_r.pressed = true; key_r.last_time = now; break;
          case 'f': key_f.pressed = true; key_f.last_time = now; break;
          case 't': key_t.pressed = true; key_t.last_time = now; break;
          case 'g': key_g.pressed = true; key_g.last_time = now; break;
          case ' ': // Stop all motion: clear all key states
            arrow_up.pressed = arrow_down.pressed = arrow_left.pressed = arrow_right.pressed = false;
            key_w.pressed = key_s.pressed = key_a.pressed = key_d.pressed = false;
            key_r.pressed = key_f.pressed = key_t.pressed = key_g.pressed = false;
            break;
          case 'q':
            running_ = false;
            rclcpp::shutdown();
            break;
          default:
            cout << "Unknown command: " << static_cast<char>(ch) << "\n";
            break;
        }
      }

      for (auto& effect : letter_effects) {
        if (effect.key->pressed) {
          *(effect.target) += effect.multiplier;
        }
      }

      // Update the atomic values for publishing
      force_x_.store(current_force_x);
      force_y_.store(current_force_y);
      force_z_.store(current_force_z);
      torque_x_.store(current_torque_x);
      torque_y_.store(current_torque_y);
      torque_z_.store(current_torque_z);

      std::this_thread::sleep_for(10ms);
    }
  }

  void publishLoop() const {
    rclcpp::Rate rate(PUBLISH_RATE);
    while (rclcpp::ok() && running_) {
      auto current_msg = geometry_msgs::msg::Wrench();
      current_msg.force.x  = force_x_.load();
      current_msg.force.y  = force_y_.load();
      current_msg.force.z  = force_z_.load();
      current_msg.torque.x = torque_x_.load();
      current_msg.torque.y = torque_y_.load();
      current_msg.torque.z = torque_z_.load();

      publisher_->publish(current_msg);
      rate.sleep();
    }
  }
};

int main(const int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubjugatorKeyboardControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
