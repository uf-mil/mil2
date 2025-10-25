#include "subjugator_keyboard_control/SubjugatorKeyboardControl.h"

#include <poll.h>
#include <unistd.h>

using namespace std::chrono_literals;
using namespace std::chrono;
using std::atomic;
using std::thread;

constexpr int PUBLISH_RATE = 10;  // Hertz

SubjugatorKeyboardControl::SubjugatorKeyboardControl()
  : Node("subjugator_keyboard_control")
  , force_x_(0.0)
  , force_y_(0.0)
  , force_z_(0.0)
  , torque_x_(0.0)
  , torque_y_(0.0)
  , torque_z_(0.0)
  , running_(true)
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_wrench", PUBLISH_RATE);
    this->declare_parameter("linear_speed", 100.0);
    this->declare_parameter("angular_speed", 100.0);
    base_linear_ = this->get_parameter("linear_speed").as_double();
    base_angular_ = this->get_parameter("angular_speed").as_double();

    initTerminal();

    // Start keyboard and publisher threads
    keyboard_thread_ = thread(&SubjugatorKeyboardControl::keyboardLoop, this);
    publisher_thread_ = thread(&SubjugatorKeyboardControl::publishLoop, this);

    RCLCPP_INFO(this->get_logger(), "Publishing wrench to \\cmd_wrench topic.");
    RCLCPP_INFO(this->get_logger(), R"(
    Subjugator Keyboard Control:
        w           : +x force
        s           : -x force
        a           : +y force
        d           : -y force
        x           : +z force
        z           : -z force
        Up          : pitch up   (-torque y)
        Down        : pitch down (+torque y)
        Right       : yaw right  (-torque z)
        Left        : yaw left   (+torque z)
        e           : roll left  (-torque x)
        r           : roll right (+torque x)
        m           : Spawn marble
        q           : Quit
    )");
}

SubjugatorKeyboardControl::~SubjugatorKeyboardControl()
{
    running_ = false;
    if (keyboard_thread_.joinable())
        keyboard_thread_.join();
    if (publisher_thread_.joinable())
        publisher_thread_.join();
    restoreTerminal();
}

void SubjugatorKeyboardControl::initTerminal()
{
    tcgetattr(STDIN_FILENO, &old_terminal_settings_);
    termios new_settings = old_terminal_settings_;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    new_settings.c_cc[VMIN] = 0;   // Set non-blocking input
    new_settings.c_cc[VTIME] = 1;  // 0.1 second timeout
    // These flags are necessary so doesn't always return -1
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
    terminal_initialized_ = true;
}

void SubjugatorKeyboardControl::restoreTerminal() const
{
    if (terminal_initialized_)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_settings_);
    }
}

void SubjugatorKeyboardControl::keyboardLoop()
{
    KeyState arrow_up, arrow_down, arrow_left, arrow_right;
    KeyState key_e, key_r, key_w, key_s, key_a, key_d, key_z, key_x;

    double current_force_x = 0.0, current_force_y = 0.0, current_force_z = 0.0;
    double current_torque_x = 0.0, current_torque_y = 0.0, current_torque_z = 0.0;

    std::vector<KeyState*> keys = { &arrow_up, &arrow_down, &arrow_left, &arrow_right, &key_e, &key_r,
                                    &key_w,    &key_s,      &key_a,      &key_d,       &key_z, &key_x };

    AxisEffect letter_effects[] = {
        { &arrow_up, -base_angular_, &current_torque_y },    { &arrow_down, base_angular_, &current_torque_y },
        { &arrow_right, -base_angular_, &current_torque_z }, { &arrow_left, base_angular_, &current_torque_z },
        { &key_e, -base_angular_, &current_torque_x },       { &key_r, base_angular_, &current_torque_x },
        { &key_w, base_linear_, &current_force_x },          { &key_s, -base_linear_, &current_force_x },
        { &key_a, base_linear_, &current_force_y },          { &key_d, -base_linear_, &current_force_y },
        { &key_x, base_linear_, &current_force_z },          { &key_z, -base_linear_, &current_force_z },
    };
    auto const timeout = milliseconds(500);

    while (running_)
    {
        auto now = steady_clock::now();

        // Update key timeouts first?
        for (auto key : keys)
        {
            if (now - key->last_time > timeout)
                key->pressed = false;
        }

        // Reset every time
        current_force_x = 0.0, current_force_y = 0.0, current_force_z = 0.0;
        current_torque_x = 0.0, current_torque_y = 0.0, current_torque_z = 0.0;

        // need to prevent blocking so that we can get real time input
        pollfd fd;
        fd.fd = STDIN_FILENO;  // terminal
        fd.events = POLLIN;    // only looking where there is input

        int ret = poll(&fd, 1, 0);  // 10 ms timeout to see if there is input
        if (ret > 0)
        {
            int ch = getchar();
            if (ch == 27)
            {  // Possible arrow key escape sequence
                if (getchar() == 91)
                {
                    int ch3 = getchar();
                    if (ch3 == 'A')
                    {
                        arrow_up.pressed = true;
                        arrow_up.last_time = now;
                    }
                    else if (ch3 == 'B')
                    {
                        arrow_down.pressed = true;
                        arrow_down.last_time = now;
                    }
                    else if (ch3 == 'C')
                    {
                        arrow_right.pressed = true;
                        arrow_right.last_time = now;
                    }
                    else if (ch3 == 'D')
                    {
                        arrow_left.pressed = true;
                        arrow_left.last_time = now;
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Unknown escape sequence");
                    }
                }
            }
            else
            {
                // Process normal keys:
                switch (ch)
                {
                    case 'w':
                        key_w.pressed = true;
                        key_w.last_time = now;
                        break;
                    case 's':
                        key_s.pressed = true;
                        key_s.last_time = now;
                        break;
                    case 'a':
                        key_a.pressed = true;
                        key_a.last_time = now;
                        break;
                    case 'd':
                        key_d.pressed = true;
                        key_d.last_time = now;
                        break;
                    case 'e':
                        key_e.pressed = true;
                        key_e.last_time = now;
                        break;
                    case 'r':
                        key_r.pressed = true;
                        key_r.last_time = now;
                        break;
                    case 'z':
                        key_z.pressed = true;
                        key_z.last_time = now;
                        break;
                    case 'x':
                        key_x.pressed = true;
                        key_x.last_time = now;
                        break;
                    case 'm':
                    {
                        std::cout << "[SubjugatorKeyboardControl] Spawning marble..." << std::endl;
                        std_msgs::msg::String keypress_msg;
                        keypress_msg.data = "m";
                        keypress_publisher_->publish(keypress_msg);
                        std::cout << "[SubjugatorKeyboardControl] Marble spawn request sent." << std::endl;
                        break;
                    }
                    case 'q':
                        running_ = false;
                        rclcpp::shutdown();
                        break;
                    default:
                        RCLCPP_INFO(this->get_logger(), "Unknown command: %c", static_cast<char>(ch));
                        break;
                }
            }
        }

        for (auto& effect : letter_effects)
        {
            if (effect.key->pressed)
            {
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

void SubjugatorKeyboardControl::publishLoop() const
{
    rclcpp::Rate rate(PUBLISH_RATE);
    while (rclcpp::ok() && running_)
    {
        auto current_msg = geometry_msgs::msg::Wrench();
        current_msg.force.x = force_x_.load();
        current_msg.force.y = force_y_.load();
        current_msg.force.z = force_z_.load();
        current_msg.torque.x = torque_x_.load();
        current_msg.torque.y = torque_y_.load();
        current_msg.torque.z = torque_z_.load();

        publisher_->publish(current_msg);
        rate.sleep();
    }
}
