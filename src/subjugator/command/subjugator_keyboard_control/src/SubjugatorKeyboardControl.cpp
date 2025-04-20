#include "subjugator_keyboard_control/SubjugatorKeyboardControl.h"

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
    // Initialize SDL video for real key events
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        RCLCPP_ERROR(get_logger(), "SDL_Init Error: %s", SDL_GetError());
        return;
    }
    window_ = SDL_CreateWindow("subj_ctl", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 600, 400,
                               SDL_WINDOW_SHOWN  // <-- must be SHOWN, not HIDDEN
    );
    SDL_RaiseWindow(window_);

    // Enable raw terminal mode to disable echo and line buffering
    if (tcgetattr(STDIN_FILENO, &old_tio_) == 0)
    {
        termios new_tio = old_tio_;
        new_tio.c_lflag &= ~(ICANON | ECHO);  // non-canonical, no echo
        new_tio.c_cc[VMIN] = 0;               // non-blocking read
        new_tio.c_cc[VTIME] = 1;              // 0.1s timeout
        if (tcsetattr(STDIN_FILENO, TCSANOW, &new_tio) == 0)
        {
            termios_initialized_ = true;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Failed to set raw terminal mode");
        }
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Failed to get terminal attributes");
    }
}

void SubjugatorKeyboardControl::restoreTerminal() const
{
    if (window_)
    {
        SDL_DestroyWindow(window_);
    }
    SDL_Quit();

    if (termios_initialized_)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
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

    SDL_Event e;
    while (running_)
    {
        auto now = steady_clock::now();

        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_KEYDOWN && !e.key.repeat)
            {
                switch (e.key.keysym.sym)
                {
                    case SDLK_w:
                        key_w.pressed = true;
                        key_w.last_time = now;
                        break;
                    case SDLK_s:
                        key_s.pressed = true;
                        key_s.last_time = now;
                        break;
                    case SDLK_a:
                        key_a.pressed = true;
                        key_a.last_time = now;
                        break;
                    case SDLK_d:
                        key_d.pressed = true;
                        key_d.last_time = now;
                        break;
                    case SDLK_e:
                        key_e.pressed = true;
                        key_e.last_time = now;
                        break;
                    case SDLK_r:
                        key_r.pressed = true;
                        key_r.last_time = now;
                        break;
                    case SDLK_x:
                        key_x.pressed = true;
                        key_x.last_time = now;
                        break;
                    case SDLK_z:
                        key_z.pressed = true;
                        key_z.last_time = now;
                        break;
                    case SDLK_UP:
                        arrow_up.pressed = true;
                        arrow_up.last_time = now;
                        break;
                    case SDLK_DOWN:
                        arrow_down.pressed = true;
                        arrow_down.last_time = now;
                        break;
                    case SDLK_RIGHT:
                        arrow_right.pressed = true;
                        arrow_right.last_time = now;
                        break;
                    case SDLK_LEFT:
                        arrow_left.pressed = true;
                        arrow_left.last_time = now;
                        break;
                    case SDLK_q:
                        running_ = false;
                        rclcpp::shutdown();
                        break;
                    default:
                        break;
                }
            }
            else if (e.type == SDL_KEYUP)
            {
                switch (e.key.keysym.sym)
                {
                    case SDLK_w:
                        key_w.pressed = false;
                        break;
                    case SDLK_s:
                        key_s.pressed = false;
                        break;
                    case SDLK_a:
                        key_a.pressed = false;
                        break;
                    case SDLK_d:
                        key_d.pressed = false;
                        break;
                    case SDLK_e:
                        key_e.pressed = false;
                        break;
                    case SDLK_r:
                        key_r.pressed = false;
                        break;
                    case SDLK_x:
                        key_x.pressed = false;
                        break;
                    case SDLK_z:
                        key_z.pressed = false;
                        break;
                    case SDLK_UP:
                        arrow_up.pressed = false;
                        break;
                    case SDLK_DOWN:
                        arrow_down.pressed = false;
                        break;
                    case SDLK_RIGHT:
                        arrow_right.pressed = false;
                        break;
                    case SDLK_LEFT:
                        arrow_left.pressed = false;
                        break;
                    default:
                        break;
                }
            }
        }
        // Reset every time
        current_force_x = 0.0, current_force_y = 0.0, current_force_z = 0.0;
        current_torque_x = 0.0, current_torque_y = 0.0, current_torque_z = 0.0;

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
