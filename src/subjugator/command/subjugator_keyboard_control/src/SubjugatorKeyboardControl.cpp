#include "subjugator_keyboard_control/SubjugatorKeyboardControl.h"

#include <poll.h>
#include <unistd.h>

#include <cmath>
#include <functional>

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
    // Create separate publishers for wrenches and goal poses
    wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("cmd_wrench", PUBLISH_RATE);
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("goal_pose", PUBLISH_RATE);

    this->declare_parameter("linear_speed", 100.0);
    this->declare_parameter("angular_speed", 100.0);
    base_linear_ = this->get_parameter("linear_speed").as_double();
    base_angular_ = this->get_parameter("angular_speed").as_double();

    // Create a subscription to odometry/filtered
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, std::bind(&SubjugatorKeyboardControl::odometryCallback, this, std::placeholders::_1));
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
        t           : toggle control mode (raw/smart)
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

void SubjugatorKeyboardControl::odometryCallback(nav_msgs::msg::Odometry::SharedPtr const msg)
{
    // store a copy in last_odom_
    last_odom_ = *msg;  // copy pointwise
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
    KeyState key_e, key_r, key_w, key_s, key_a, key_d, key_z, key_x, key_t;

    double current_force_x = 0.0, current_force_y = 0.0, current_force_z = 0.0;
    double current_torque_x = 0.0, current_torque_y = 0.0, current_torque_z = 0.0;

    std::vector<KeyState *> keys = { &arrow_up, &arrow_down, &arrow_left, &arrow_right, &key_e, &key_r, &key_w,
                                     &key_s,    &key_a,      &key_d,      &key_z,       &key_x, &key_t };

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
                    case 't':
                        if (use_smart_control_)
                        {
                            RCLCPP_INFO(this->get_logger(), "Using raw control");
                            use_smart_control_ = false;
                        }
                        else
                        {
                            RCLCPP_INFO(this->get_logger(), "Using smart control");
                            use_smart_control_ = true;
                        }
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

        for (auto &effect : letter_effects)
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

geometry_msgs::msg::Pose SubjugatorKeyboardControl::rotateVectorByQuat(nav_msgs::msg::Odometry const &ref, double dx,
                                                                       double dy, double dz)
{
    auto const &q = ref.pose.pose.orientation;

    // Standard quaternion-vector multiplication: v' = q * v * q^-1
    // For pure vector v = (dx,dy,dz), quaternion multiplication simplifies to:
    double x = dx * (1 - 2 * q.y * q.y - 2 * q.z * q.z) + dy * (2 * q.x * q.y - 2 * q.z * q.w) +
               dz * (2 * q.x * q.z + 2 * q.y * q.w);

    double y = dx * (2 * q.x * q.y + 2 * q.z * q.w) + dy * (1 - 2 * q.x * q.x - 2 * q.z * q.z) +
               dz * (2 * q.y * q.z - 2 * q.x * q.w);

    double z = dx * (2 * q.x * q.z - 2 * q.y * q.w) + dy * (2 * q.y * q.z + 2 * q.x * q.w) +
               dz * (1 - 2 * q.x * q.x - 2 * q.y * q.y);

    geometry_msgs::msg::Pose out;
    out.position.x = x;
    out.position.y = y;
    out.position.z = z;

    // Orientation is irrelevant here; we just need the rotated delta
    out.orientation.x = 0.0;
    out.orientation.y = 0.0;
    out.orientation.z = 0.0;
    out.orientation.w = 1.0;

    return out;
}

geometry_msgs::msg::Pose SubjugatorKeyboardControl::createGoalPose() const
{
    // Movement increments (meters and radians)
    double x_movement = 0.001 * force_x_.load();
    double y_movement = 0.001 * force_y_.load();
    double z_movement = 0.001 * force_z_.load();
    double roll = 0.01 * torque_x_.load();
    double pitch = 0.01 * torque_y_.load();
    double yaw = 0.01 * torque_z_.load();

    // Convert roll, pitch, yaw to quaternion (ZYX order)
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    double qw = cy * cp * cr + sy * sp * sr;
    double qx = cy * cp * sr - sy * sp * cr;
    double qy = cy * sp * cr + sy * cp * sr;
    double qz = sy * cp * cr - cy * sp * sr;

    geometry_msgs::msg::Pose goal;

    // Rotate the delta vector by the current orientation
    auto rel_rotated = rotateVectorByQuat(last_odom_, x_movement, y_movement, z_movement);
    // Add rotated delta to the current pose
    auto current_x = last_odom_.pose.pose.position.x;
    auto current_y = last_odom_.pose.pose.position.y;
    auto current_z = last_odom_.pose.pose.position.z;
    goal.position.x = current_x + rel_rotated.position.x;
    goal.position.y = current_y + rel_rotated.position.y;
    goal.position.z = current_z + rel_rotated.position.z;

    // Combine current orientation with the incremental rotation
    auto const &c = last_odom_.pose.pose.orientation;
    goal.orientation.x = c.w * qx + c.x * qw + c.y * qz - c.z * qy;
    goal.orientation.y = c.w * qy - c.x * qz + c.y * qw + c.z * qx;
    goal.orientation.z = c.w * qz + c.x * qy - c.y * qx + c.z * qw;
    goal.orientation.w = c.w * qw - c.x * qx - c.y * qy - c.z * qz;

    return goal;
}

void SubjugatorKeyboardControl::publishLoop() const
{
    rclcpp::Rate rate(PUBLISH_RATE);
    while (rclcpp::ok() && running_)
    {
        if (!use_smart_control_)
        {
            auto current_msg = geometry_msgs::msg::Wrench();
            current_msg.force.x = force_x_.load();
            current_msg.force.y = force_y_.load();
            current_msg.force.z = force_z_.load();
            current_msg.torque.x = torque_x_.load();
            current_msg.torque.y = torque_y_.load();
            current_msg.torque.z = torque_z_.load();

            wrench_publisher_->publish(current_msg);
            rate.sleep();
        }
        else
        {
            auto current_msg = this->createGoalPose();

            pose_publisher_->publish(current_msg);
            rate.sleep();
        }
    }
}
