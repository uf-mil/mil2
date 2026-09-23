#include <atomic>
#include <thread>

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <rclcpp/rclcpp.hpp>

#include "navigator_controller/client.h"

namespace navigator_controller
{

class KeyboardClient : public Client
{
  public:
    KeyboardClient() : Client("navigator_keyboard_controller")
    {
        declare_parameter("force_x", 100.0);
        declare_parameter("force_y", 100.0);
        declare_parameter("torque_z", 100.0);
        declare_parameter("publish_frequency", 50.0);

        force_x_ = get_parameter("force_x").as_double();
        force_y_ = get_parameter("force_y").as_double();
        torque_z_ = get_parameter("torque_z").as_double();

        param_cb_ = add_on_set_parameters_callback(
            [this](std::vector<rclcpp::Parameter> const& params)
            {
                for (auto const& p : params)
                {
                    if (p.get_name() == "force_x")
                        force_x_ = p.as_double();
                    else if (p.get_name() == "force_y")
                        force_y_ = p.as_double();
                    else if (p.get_name() == "torque_z")
                        torque_z_ = p.as_double();
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        double freq = get_parameter("publish_frequency").as_double();
        auto period = std::chrono::duration<double>(1.0 / freq);
        timer_ = create_wall_timer(
            period, [this]() { control_wrench(active(fx_, fx_t_), active(fy_, fy_t_), 0, 0, 0, active(tz_, tz_t_)); });
    }

    void w_pressed()
    {
        fx_ = static_cast<float>(force_x_);
        fx_t_ = now_ns();
    }
    void s_pressed()
    {
        fx_ = -static_cast<float>(force_x_);
        fx_t_ = now_ns();
    }
    void a_pressed()
    {
        fy_ = static_cast<float>(force_y_);
        fy_t_ = now_ns();
    }
    void d_pressed()
    {
        fy_ = -static_cast<float>(force_y_);
        fy_t_ = now_ns();
    }
    void left_arrow_pressed()
    {
        tz_ = static_cast<float>(torque_z_);
        tz_t_ = now_ns();
    }
    void right_arrow_pressed()
    {
        tz_ = -static_cast<float>(torque_z_);
        tz_t_ = now_ns();
    }

  private:
    static constexpr int64_t KEY_TIMEOUT_NS = 150'000'000;  // 150 ms

    static int64_t now_ns()
    {
        return std::chrono::steady_clock::now().time_since_epoch().count();
    }

    static float active(std::atomic<float> const& val, std::atomic<int64_t> const& t)
    {
        return (now_ns() - t.load()) < KEY_TIMEOUT_NS ? val.load() : 0.0f;
    }

    double force_x_{ 100.0 };
    double force_y_{ 100.0 };
    double torque_z_{ 100.0 };
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::atomic<float> fx_{ 0.0f };
    std::atomic<int64_t> fx_t_{ 0 };
    std::atomic<float> fy_{ 0.0f };
    std::atomic<int64_t> fy_t_{ 0 };
    std::atomic<float> tz_{ 0.0f };
    std::atomic<int64_t> tz_t_{ 0 };
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigator_controller

void run_ui(std::shared_ptr<navigator_controller::KeyboardClient> client)
{
    using namespace ftxui;

    std::vector<std::string> const help_lines = {
        "Move Forward:         w          ", "Move Backward:        s          ", "Move Port:            a          ",
        "Move Starboard:       d          ", "Yaw Counterclockwise: arrow left ", "Yaw Clockwise:        arrow right",
        "Quit:                 q          ",
    };

    auto screen = ScreenInteractive::Fullscreen();
    auto exit = screen.ExitLoopClosure();

    auto renderer = Renderer(
        [&help_lines]()
        {
            Elements rows;
            for (auto const& line : help_lines)
                rows.push_back(text(line));

            return vbox({
                text(" Navigator Keyboard Controller ") | bold | center,
                separator(),
                vbox(rows) | border,
            });
        });

    auto component = CatchEvent(renderer,
                                [=](Event event) -> bool
                                {
                                    if (event == Event::Character('q'))
                                    {
                                        rclcpp::shutdown();
                                        exit();
                                        return true;
                                    }
                                    if (event == Event::Character('w'))
                                    {
                                        client->w_pressed();
                                        return true;
                                    }
                                    if (event == Event::Character('s'))
                                    {
                                        client->s_pressed();
                                        return true;
                                    }
                                    if (event == Event::Character('a'))
                                    {
                                        client->a_pressed();
                                        return true;
                                    }
                                    if (event == Event::Character('d'))
                                    {
                                        client->d_pressed();
                                        return true;
                                    }
                                    if (event == Event::ArrowLeft)
                                    {
                                        client->left_arrow_pressed();
                                        return true;
                                    }
                                    if (event == Event::ArrowRight)
                                    {
                                        client->right_arrow_pressed();
                                        return true;
                                    }
                                    return false;
                                });

    screen.Loop(component);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<navigator_controller::KeyboardClient>();

    std::thread ui_thread(run_ui, node);
    rclcpp::spin(node);
    rclcpp::shutdown();

    ui_thread.join();
    return 0;
}
