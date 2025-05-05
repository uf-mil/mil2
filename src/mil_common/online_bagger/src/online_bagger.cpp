#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/dom/elements.hpp"
#include "ftxui/component/loop.hpp"

#include "online_bagger/client.hpp"
#include "mil_tools/tui/dialog.hpp"

#include <future>

#include <boost/program_options.hpp>

using namespace ftxui;
using namespace online_bagger;

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

class Tui
{
    public:
    Tui()
    {

    }

    ~Tui()
    {

    }

    void spin(std::shared_ptr<online_bagger::Client> client)
    {
        // Head
        Component ui_head = Renderer([]{return text("Online Bagger") | bold | center;});
        // Body
        std::shared_ptr<TopicList> ui_topic_list = std::make_shared<TopicList>();
        
        Component ui_body = ui_topic_list | vscroll_indicator | frame ;

        // Bottom
        auto on_quit = [this]{screen.Exit();};
        auto on_bag = [this, ui_topic_list, client]{
            bag_progress = 0.0f;

            mil_msgs::action::BagOnline::Goal goal; 
            goal.topics = ui_topic_list->get_selected();
            goal.bag_time = 1.0f;

            if(!ask_bag_detail(goal.bag_name, goal.bag_time))
                return;

            auto goal_options = rclcpp_action::Client<mil_msgs::action::BagOnline>::SendGoalOptions();
            goal_options.goal_response_callback = [this](Client::BagOnlineGoalHandle::SharedPtr goal_handle){
                if(!goal_handle)
                {
                    screen.Post([this]{show_message("Failed", "Request is rejected by the online bagger server");});
                }
            };
            goal_options.result_callback = [this, client = client](const Client::BagOnlineGoalHandle::WrappedResult& result){
                std::string message = result.result->success ? "Succuessfully save bag file to " + result.result->filename :
                                                            "Failed to bag topics: " + result.result->status;
                screen.Post([this, message = std::move(message), success = result.result->success]{
                    show_message(success ? "Success" : "Failed", message);
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
    float bag_progress = 0.0f;

    bool ask_bag_detail(std::string& name, float& time)
    {
        mil_tools::tui::Dialog dialog;
        std::string time_input = std::to_string(time);
        Component ui_name_input = Input(&name, InputOption::Default());
        Component ui_time_input = Input(&time_input, InputOption::Default()) | CatchEvent([](Event event) {
            return event.is_character() && !(std::isdigit(event.character()[0]) || event.character()[0] == '.');
        });
        
        Component content = Renderer(Container::Vertical({ui_name_input, ui_time_input}),[=]{
            return gridbox({
                {text("Bag File Name ") | vcenter, ui_name_input->Render() | border},
                {text("Bag Time ") | vcenter, ui_time_input->Render() | border}
            });
        }) | vscroll_indicator | frame;

        if(dialog.exec("Bag details", content) == -1)
            return false;

        try
        {
            time = std::stof(time_input);
        }
        catch(const std::exception& e)
        {
            return false;   
        }

        return true;
    }

    void show_message(const std::string& title, const std::string& message)
    {
        mil_tools::tui::Dialog dialog;
        dialog.exec(title, Renderer([&]{return paragraph(message);}));
    }
};

namespace po = boost::program_options;

void show_help_message()
{

}

int main(int argc, char** argv) 
{
    std::string subcommand;
    std::vector<std::string> options;

    po::options_description global_opts("Global");
    global_opts.add_options()
        ("subcommand", po::value<std::string>(&subcommand), "Subcommand")
        ("options", po::value<std::vector<std::string>>(&options), "Args for subcommand");

    po::positional_options_description pos;
    pos.add("subcommand", 1).add("options", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv)
                .options(global_opts)
                .positional(pos)
                .allow_unregistered()
                .run(),
            vm);
    po::notify(vm);

    enum class Mode {
        TUI,
        HELP,
        BAG,
        LIST
    };

    const std::unordered_map<std::string, Mode> subcmd_mapping = {
        {"tui",  Mode::TUI},
        {"help", Mode::HELP},
        {"bag",  Mode::BAG},
        {"list", Mode::LIST}
    };

    auto it = subcmd_mapping.find(subcommand);
    if(it == subcmd_mapping.end())
    {
        show_help_message();
        return 1;
    }

    Mode mode = it -> second;
    rclcpp::init(1, argv);
    std::shared_ptr<online_bagger::Client> client = std::make_shared<online_bagger::Client>();

    if(mode == Mode::HELP)
    {
        show_help_message();
    }
    else if(mode == Mode::LIST)
    {
        auto future = client->get_bag_topics([](rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture future){
            for(const std::string& topic : future.get()->topics)
            {
                std::cout << topic << std::endl;
            }
            rclcpp::shutdown();
        });

        if(rclcpp::spin_until_future_complete(client, future, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::TIMEOUT)
            std::cout << "Online bagger server doesn't exist.\n";
    }
    else if(mode == Mode::BAG)
    {   
        auto future = client->get_bag_topics([&]([[maybe_unused]]rclcpp::Client<mil_msgs::srv::BagTopics>::SharedFuture future){});

        if(rclcpp::spin_until_future_complete(client, future, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS)
        {
            mil_msgs::action::BagOnline::Goal goal;
            goal.topics = future.get()->topics;
            goal.bag_time = 1.0f;

            po::options_description bag_opts("bag options");
            bag_opts.add_options()
                ("name,n", po::value<std::string>(&goal.bag_name), "Bag name")
                ("time,t", po::value<float>(&goal.bag_time), "Time in seconds")
                ("topics,p", po::value<std::vector<std::string>>(&goal.topics)->multitoken(), "Topics to bag");

            po::variables_map bag_vm;
            po::store(po::command_line_parser(options)
                        .options(bag_opts)
                        .run(),
                    bag_vm);
            po::notify(bag_vm);

            auto goal_options = rclcpp_action::Client<mil_msgs::action::BagOnline>::SendGoalOptions();
            goal_options.goal_response_callback = [](online_bagger::Client::BagOnlineGoalHandle::SharedPtr goal_handle){
                if(!goal_handle)
                {
                    std::cout << "Failed to bag: Request is rejected by the online bagger server.\n";
                }
            };
            goal_options.result_callback = [&](const online_bagger::Client::BagOnlineGoalHandle::WrappedResult& result){
                std::string message = result.result->success ? "Succuessfully save bag file to " + result.result->filename :
                                                            "Failed to bag topics: " + result.result->status;
                std::cout << message << ".\n";
                client->finish_bagging();
                rclcpp::shutdown();
            };
            goal_options.feedback_callback = [&](
                [[maybe_unused]]online_bagger::Client::BagOnlineGoalHandle::SharedPtr goal_handle, 
                const std::shared_ptr<const mil_msgs::action::BagOnline::Feedback> feedback){
                std::cout << "Bagging " << feedback->progress << "%\n";
            };

            auto bag_future = client->start_bagging(goal, goal_options);

            rclcpp::spin(client);

            std::thread thread([&]{
                while(bag_future.wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
                {
                    if(client->get_state() == Client::State::Waiting)
                    {
                        std::cout << "Lost connection with the online bagger server.\n";
                        rclcpp::shutdown();
                        break;
                    }
                }
            });

            thread.join();
        }
        else
        {
            std::cout << "Online bagger server doesn't exist.\n";
        }
    }
    else if(mode == Mode::TUI)
    {
        std::thread thread([=]{
            Tui tui;
            tui.spin(client);
            rclcpp::shutdown();
        });

        rclcpp::spin(client);
        thread.join();
    }

    rclcpp::shutdown();
    return 0;
}

