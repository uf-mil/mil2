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

class UIScreen
{
    public:
    UIScreen(Component ui):
        loop(&screen, ui)
    {

    }

    ~UIScreen()
    {
    }

    void shutdown()
    {
        screen.Exit();   
    }

    bool ok()
    {
        return !loop.HasQuitted();
    }

    void spin_once()
    {
        loop.RunOnce();
    }

    void refresh()
    {
        screen.PostEvent(Event::Custom);
    }

    private:
    ScreenInteractive screen = ScreenInteractive::Fullscreen();
    Loop loop;
};

class Client: public rclcpp::Node
{
    public:
    Client():rclcpp::Node("online_bagger_client")
    {
        using namespace ftxui;

        action_client = rclcpp_action::create_client<mil_msgs::action::BagOnline>(this, BAG_ACTION_NAME);
        srv_client = create_client<mil_msgs::srv::BagTopics>(TOPIC_SERVICE_NAME);

        // Head
        Component ui_head = Renderer([]{return text("Online Bagger") | bold | center | flex;});
        // Body
        Component ui_name_input = Input(&name_input, InputOption::Default());
        Component ui_time_input = Input(&time_input, InputOption::Default()) | CatchEvent([](Event event) {
            return event.is_character() && !(std::isdigit(event.character()[0]) || event.character()[0] == '.');
          });
        Component ui_body = Renderer(Container::Vertical({
            ui_topic_list, ui_name_input, ui_time_input
            }),[=]{
                return vbox({
                    text("Bag Topics") | vcenter,
                    ui_topic_list->Render() | border,
                    text("Bag File Name ") | vcenter, 
                    ui_name_input->Render() | border,
                    text("Bag Time ") | vcenter, 
                    ui_time_input->Render() | border
                }) | vscroll_indicator | frame;
        });

        // Bottom
        auto on_quit = [&]{rclcpp::shutdown();};
        auto on_bag = [&]{
            bag_progress = 0.0f;
            mil_msgs::action::BagOnline::Goal goal; 
            goal.bag_name = name_input;
            goal.topics = ui_topic_list->get_selected();
            goal.bag_time = std::stof(time_input);

            auto goal_options = rclcpp_action::Client<mil_msgs::action::BagOnline>::SendGoalOptions();
            goal_options.goal_response_callback = std::bind(&Client::handle_reponse, this, std::placeholders::_1);
            goal_options.result_callback = std::bind(&Client::handle_result, this, std::placeholders::_1);
            goal_options.feedback_callback = std::bind(&Client::handle_feedback, this, std::placeholders::_1, std::placeholders::_2);
            action_client->async_send_goal(goal, goal_options);
        };
        auto on_refresh = [=]{
            auto request = std::make_shared<mil_msgs::srv::BagTopics::Request>();
            srv_client->async_send_request(request, [&](
                rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture future){
                ui_topic_list->refresh(std::move(future.get()->topics));
            });
        };

        Component ui_status_bar = Renderer([&]{
            if(client_state == ClientState::Waiting)
                return hbox({
                    text("Connecting to bag server "), 
                    spinner(15, now().nanoseconds() / 200'000'000)
                });
            else if(client_state == ClientState::Ready)
                return filler();
            else
                return gauge(bag_progress);
        });

        Component ui_bottom_bar = Container::Horizontal({
            ui_status_bar | vcenter | xflex,
            Button("Quit", on_quit, ButtonOption::Border()) | align_right,
            Maybe(Button("Refresh", on_refresh, ButtonOption::Border()), [&]{
                return client_state != ClientState::Waiting;
            }) | align_right,
            Maybe(Button("Bag", on_bag, ButtonOption::Border()), [&]{
                return client_state == ClientState::Ready;
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

        screens.emplace(main_ui);

        ui_timer = create_wall_timer(std::chrono::milliseconds(20), std::bind(&Client::refresh_ui, this));
        alive_timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Client::is_alive, this));
    }
    ~Client()
    {

    }
    private:

    using BagOnlineGoalHandle = rclcpp_action::ClientGoalHandle<mil_msgs::action::BagOnline>;

    enum class ClientState
    {
        Waiting,
        Ready,
        Bagging
    }client_state = ClientState::Waiting;

    float bag_progress = 0.0f;
    std::string result_string;
    rclcpp_action::Client<mil_msgs::action::BagOnline>::SharedPtr action_client;
    rclcpp::Client<mil_msgs::srv::BagTopics>::SharedPtr srv_client;
    rclcpp::TimerBase::SharedPtr ui_timer;
    rclcpp::TimerBase::SharedPtr alive_timer;
    std::shared_ptr<TopicList> ui_topic_list = std::make_shared<TopicList>();
    
    std::stack<UIScreen> screens;

    std::string name_input = "bag";
    std::string time_input = "1";

    void refresh_ui()
    {
        UIScreen& screen = screens.top();
        if(screen.ok())
        {
            screen.spin_once();
        }
        else
        {
            screens.pop();
            if(screens.size() == 0)
                rclcpp::shutdown();
        }
    }

    void is_alive()
    {
        if(client_state == ClientState::Waiting && action_client->action_server_is_ready())
        {
            auto request = std::make_shared<mil_msgs::srv::BagTopics::Request>();
            srv_client->async_send_request(request, [&](
                rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture future){
                    client_state = ClientState::Ready;
                    ui_topic_list->refresh(std::move(future.get()->topics));
            });
        }
        else if(client_state == ClientState::Ready && !action_client->action_server_is_ready())
        {
            client_state = ClientState::Waiting;
        }
        else if(client_state == ClientState::Bagging && !action_client->action_server_is_ready())
        {
            client_state = ClientState::Waiting;
        }
        screens.top().refresh();
    }

    void handle_reponse(BagOnlineGoalHandle::SharedPtr goal_handle)
    {
        if(!goal_handle)
        {
            show_dialog("Failed", "Request is rejected by the online bagger server");
        }
    }

    void handle_feedback([[maybe_unused]]BagOnlineGoalHandle::SharedPtr goal_handle, 
        const std::shared_ptr<const mil_msgs::action::BagOnline::Feedback> feedback)
    {
        bag_progress = feedback->progress;
    }

    void handle_result(const BagOnlineGoalHandle::WrappedResult& result)
    {
        if(result.result->success)
        {
            show_dialog("Success", "Succuessfully save bag file to " + result.result->filename);
        }
        else
        {
            show_dialog("Failed", "Failed to bag topics: " + result.result->status);
        }
        
        client_state = ClientState::Ready;
    }

    void show_dialog(const std::string& title, const std::string& body)
    {
        Component dialog_ui = Container::Vertical({});
        UIScreen& screen = screens.emplace(dialog_ui | border);

        Component close_button = Button("X", [&]{
            screen.shutdown();
        }, ButtonOption::Ascii());

        dialog_ui->Add(Renderer(close_button,[=]{
            return hbox({
                text(title) | center |flex,
                close_button->Render()
            });
        }));

        dialog_ui->Add(Renderer([]{return separator();}));
        dialog_ui->Add(Renderer([=]{return paragraph(body);}));
    }
    
};
}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<online_bagger::Client> client = std::make_shared<online_bagger::Client>();
    
    rclcpp::spin(client);
    rclcpp::shutdown();

    return 0;
}
