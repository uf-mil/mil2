#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/program_options.hpp>

#include "ftxui/component/loop.hpp"
#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/dom/elements.hpp"

#include "mil_msgs/action/bag_online.hpp"
#include "mil_msgs/srv/bag_topics.hpp"
#include "online_bagger/online_bagger.h"

#include <future>
#include <stack>
#include <atomic>

namespace online_bagger
{
using namespace ftxui;

class TopicList: public ComponentBase
{
    public:
    TopicList()
    {
    }
    ~TopicList()
    {

    }

    void refresh(std::vector<std::string>&& topics)
    {
        this->topics = std::move(topics);
        DetachAllChildren();
        if(selected)
            delete[] selected;
        
        if(this->topics.size() == 0)
            return;

        future = std::async(std::launch::async, [&]{
            selected = new bool[this->topics.size()]{false};
            Components items;
            for(size_t i=0; i<this->topics.size(); i++)
            {
                items.push_back(Checkbox(this->topics[i], &selected[i]));
            }

            return Container::Vertical(items);
        });

        return;
    }

    const std::vector<std::string> get_selected()
    {
        std::vector<std::string> selected_topics;
        for(size_t i=0;i<topics.size();i++)
        {
            if(selected[i])
                selected_topics.push_back(topics[i]);
        }
        return selected_topics;
    }

    Element Render() override
    {
        if(!future.valid())
        {
            if(ChildCount() > 0)
                return ChildAt(0)->Render();
            else
                return text("No bag topics available");
        }
        else
        {
            if(future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            {
                Add(future.get());
            }

            return spinner(15, clock.now().nanoseconds() / 200'000'000);
        }
    }

    private:
    std::future<Component> future; 
    std::vector<std::string> topics;
    bool* selected = nullptr;
    rclcpp::Clock clock;
};

class Client: public rclcpp::Node
{
    public:
    using BagOnlineGoalHandle = rclcpp_action::ClientGoalHandle<mil_msgs::action::BagOnline>;

    enum class State
    {
        Waiting,
        Ready,
        Bagging
    };

    struct BagOptions
    {
        std::vector<std::string> topics;
        std::string name;
        float time;
        std::function<void(float)> on_feedback;
        std::function<void(bool,std::string&&)> on_result;
    };

    Client():rclcpp::Node("online_bagger_client")
    {
        using namespace ftxui;

        action_client = rclcpp_action::create_client<mil_msgs::action::BagOnline>(this, BAG_ACTION_NAME);
        srv_client = create_client<mil_msgs::srv::BagTopics>(TOPIC_SERVICE_NAME);

        alive_timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Client::is_alive, this));
    }
    ~Client()
    {

    }

    State get_state()
    {
        return state;
    }

    
    std::shared_future<std::shared_ptr<mil_msgs::srv::BagTopics_Response>> 
        get_bag_topics(std::function<void(rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture future)> callback)
    {
        auto request = std::make_shared<mil_msgs::srv::BagTopics::Request>();
        return srv_client->async_send_request(request, callback).future;
    }

    std::shared_future<BagOnlineGoalHandle::SharedPtr>
        start_bagging(const mil_msgs::action::BagOnline::Goal& goal,
        const rclcpp_action::Client<mil_msgs::action::BagOnline>::SendGoalOptions& options)
    {
        state = State::Bagging;
        return action_client->async_send_goal(goal, options);
    }

    void finish_bagging()
    {
        state = State::Ready;
    }

    private:

    std::atomic<State> state;
    rclcpp_action::Client<mil_msgs::action::BagOnline>::SharedPtr action_client;
    rclcpp::Client<mil_msgs::srv::BagTopics>::SharedPtr srv_client;
    rclcpp::TimerBase::SharedPtr alive_timer;

    void is_alive()
    {
        if(state == State::Waiting && action_client->action_server_is_ready())
        {
            state = State::Ready;
        }
        else if(state == State::Ready && !action_client->action_server_is_ready())
        {
            state = State::Waiting;
        }
        else if(state == State::Bagging && !action_client->action_server_is_ready())
        {
            state = State::Waiting;
        }
    }
    
};

class Tui
{
    public:
    Tui()
    {

    }

    ~Tui()
    {

    }

    void spin(std::shared_ptr<Client> client)
    {
        // Head
        Component ui_head = Renderer([]{return text("Online Bagger") | bold | center | flex;});
        // Body
        std::shared_ptr<TopicList> ui_topic_list = std::make_shared<TopicList>();
        Component ui_name_input = Input(&name_input, InputOption::Default());
        Component ui_time_input = Input(&time_input, InputOption::Default()) | CatchEvent([](Event event) {
            return event.is_character() && !(std::isdigit(event.character()[0]) || event.character()[0] == '.');
          });
        Component ui_body = Container::Vertical({
            Renderer([]{return text("Bag Topics") | vcenter;}),
            ui_topic_list | border,
            Renderer([]{return text("Bag File Name ") | vcenter;}), 
            ui_name_input | border,
            Renderer([]{return text("Bag Time ") | vcenter;}), 
            ui_time_input | border
        }) | vscroll_indicator | frame ;

        // Bottom
        auto on_quit = [this]{screen.Exit();};
        auto on_bag = [this, ui_topic_list, client]{
            bag_progress = 0.0f;

            mil_msgs::action::BagOnline::Goal goal; 
            goal.bag_name = name_input;
            goal.topics = ui_topic_list->get_selected();
            goal.bag_time = std::stof(time_input);

            auto goal_options = rclcpp_action::Client<mil_msgs::action::BagOnline>::SendGoalOptions();
            goal_options.goal_response_callback = [this](Client::BagOnlineGoalHandle::SharedPtr goal_handle){
                if(!goal_handle)
                {
                    screen.Post([this]{show_dialog("Failed", "Request is rejected by the online bagger server");});
                }
            };
            goal_options.result_callback = [this, client = client](const Client::BagOnlineGoalHandle::WrappedResult& result){
                std::string message = result.result->success ? "Succuessfully save bag file to " + result.result->filename :
                                                            "Failed to bag topics: " + result.result->status;
                screen.Post([this, message = std::move(message), success = result.result->success]{
                    show_dialog(success ? "Success" : "Failed", message);
                });
                client->finish_bagging();
            };
            goal_options.feedback_callback = [this](
                [[maybe_unused]]Client::BagOnlineGoalHandle::SharedPtr goal_handle, 
                const std::shared_ptr<const mil_msgs::action::BagOnline::Feedback> feedback){
                screen.Post([this, progress = feedback->progress]{bag_progress = progress;});
            };

            client->start_bagging(goal, goal_options);
        };

        auto on_refresh = [this, ui_topic_list, client]{
            client->get_bag_topics([&](rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture future){
                screen.Post([ui_topic_list, topics = std::move(future.get()->topics)] () mutable {
                    ui_topic_list->refresh(std::move(topics));
                });
            });
        };

        Component ui_status_bar = Renderer([this, client]{
            if(client->get_state() == Client::State::Waiting)
            {
                return hbox({
                    text("Connecting to bag server "), 
                    spinner(15, clock.now().nanoseconds() / 200'000'000)
                });
            }
            else if(client->get_state() == Client::State::Ready)
                return filler();
            else
                return gauge(bag_progress);
        });

        Component ui_bottom_bar = Container::Horizontal({
            ui_status_bar | vcenter | xflex,
            Button("Quit", on_quit, ButtonOption::Border()) | align_right,
            Maybe(Button("Refresh", on_refresh, ButtonOption::Border()), [client]{
                return client->get_state() != Client::State::Waiting;
            }) | align_right,
            Maybe(Button("Bag", on_bag, ButtonOption::Border()), [client]{
                return client->get_state() == Client::State::Ready;
            }) | align_right
        });

        
        // Main UI
        Component main_ui = Container::Vertical({
            ui_head,
            Renderer([]{return separator();}),
            ui_body | flex,
            Renderer([]{return separator();}),
            ui_bottom_bar
        });

        on_refresh();

        Loop loop(&screen, main_ui);

        std::thread refresh_thread([&]{
            while(!loop.HasQuitted())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                screen.PostEvent(Event::Custom);
            }
        });

        loop.Run();

        refresh_thread.join();
    }

    private:
    rclcpp::Clock clock;
    ScreenInteractive screen = ScreenInteractive::Fullscreen();
    std::string name_input = "Bag";
    std::string time_input = "1";
    float bag_progress = 0.0f;

    void show_dialog(const std::string& title, const std::string& body)
    {
        ScreenInteractive dialog_screen = ScreenInteractive::Fullscreen();

        Component close_button = Button("X", [&]{
            dialog_screen.Exit();
        }, ButtonOption::Ascii());

        Component dialog_ui = Container::Vertical({
            Renderer(close_button,[=]{
                return hbox({
                    text(title) | center |flex,
                    close_button->Render()
                });
            }),
            Renderer([]{return separator();}),
            Renderer([=]{return paragraph(body);})
        });

        dialog_screen.Loop(dialog_ui);
    }
};

}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<online_bagger::Client> client = std::make_shared<online_bagger::Client>();

    std::thread ui_thread([=]{
        online_bagger::Tui tui;
        tui.spin(client);
        rclcpp::shutdown();
    });
    
    rclcpp::spin(client);
    rclcpp::shutdown();

    ui_thread.join();

    return 0;
}
