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
        auto on_quit = [this]{
            exited = true;
            screen.Exit();
        };
        auto on_bag = [this, ui_topic_list, client]{
            bag_progress = 0.0f;

            Client::BagOptions options; 
            options.goal.topics = ui_topic_list->get_selected();

            if(!ask_bag_detail(options.goal.bag_name, options.goal.bag_time))
                return;

            options.on_finish = [this](Client::BagFuture future){
                screen.Post([this, future]{
                    auto& result = future.get();
                    std::string message = result.first ? "Succuessfully save bag file to " + result.second :
                                                            "Failed to bag topics: " + result.second;
                    show_message("Bag Result", message);
                });
            };
            
            options.on_progress = [this](float progress){
                bag_progress = progress;
                screen.PostEvent(Event::Custom);
            };

            client->bag(options);
        };

        auto on_refresh = [this, ui_topic_list, client]{
            client->get_bag_topics([&](Client::TopicsFuture future){
                screen.Post([ui_topic_list, topics = std::move(future.get())] () mutable {
                    ui_topic_list->refresh(std::move(topics));
                });
            });
        };

        Component ui_status_bar = Renderer([this, client]{
            if(last_state == Client::State::Waiting)
            {
                return hbox({
                    text("Connecting to bag server "), 
                    spinner(15, clock.now().nanoseconds() / 200'000'000)
                });
            }
            else if(last_state == Client::State::Ready)
            {
                return filler();
            }
            else
            {
                return hbox({
                    gauge(bag_progress),
                    filler(),
                });    
            }
        });



        Component ui_bottom_bar = Container::Horizontal({
            ui_status_bar | vcenter | flex,
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

        last_state = Client::State::Waiting;
        std::thread refresh_thread([this, client, &on_refresh]{
            while(!exited)
            {
                Client::State state = client->get_state();
                if(state != Client::State::Bagging)
                {
                    screen.PostEvent(Event::Custom);
                }
                
                if(state == Client::State::Ready && last_state == Client::State::Waiting)
                {
                    on_refresh();
                }

                last_state = state;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

        screen.Loop(main_ui);
        refresh_thread.join();
    }

    private:
    std::atomic<bool> exited = false;
    std::atomic<Client::State> last_state;
    rclcpp::Clock clock;
    ScreenInteractive screen = ScreenInteractive::Fullscreen();
    std::atomic<float> bag_progress = 0.0f;

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
        auto future = client->get_bag_topics([](Client::TopicsFuture future){
            for(const std::string& topic : future.get())
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
        auto future = client->get_bag_topics();

        if(rclcpp::spin_until_future_complete(client, future, std::chrono::seconds(2)) == rclcpp::FutureReturnCode::SUCCESS)
        {
            Client::BagOptions bag_options;
            bag_options.goal.topics = std::move(future.get());
            bag_options.goal.bag_time = 1.0f;

            po::options_description bag_opts("bag options");
            bag_opts.add_options()
                ("name,n", po::value<std::string>(&bag_options.goal.bag_name), "Bag name")
                ("time,t", po::value<float>(&bag_options.goal.bag_time), "Time in seconds")
                ("topics,p", po::value<std::vector<std::string>>(&bag_options.goal.topics)->multitoken(), "Topics to bag");

            po::variables_map bag_vm;
            po::store(po::command_line_parser(options)
                        .options(bag_opts)
                        .run(),
                    bag_vm);
            po::notify(bag_vm);

            bag_options.on_finish = [&](Client::BagFuture future){
                auto& result = future.get();
                std::string message =  result.first ? "Succuessfully save bag file to " + result.second :
                                                            "Failed to bag topics: " + result.second;
                std::cout << '\n' << message << ".\n";
                rclcpp::shutdown();
            };

            bag_options.on_progress = [&](float progress){
                std::cout << "Bagging\t" <<  static_cast<int>(progress * 100) << "%\r";
            };

            auto bag_future = client->bag(bag_options);

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

            rclcpp::spin(client);

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

