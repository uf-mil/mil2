#include <filesystem>

#include <boost/dll.hpp>
#include <boost/function.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mil_preflight/plugin.h"

namespace mil_preflight
{

class PseudoAction : public Action
{
  public:
    PseudoAction(std::string&& name, std::vector<std::string> parameters)
      : name_(std::move(name)), parameters_(std::move(parameters))
    {
    }

    ~PseudoAction()
    {
    }

    std::string const& getName() const final
    {
        return name_;
    }
    std::vector<std::string> const& getParameters() const final
    {
        return parameters_;
    };
    void onStart() final
    {
    }

    void onFinish(bool success, std::string&& summery) final
    {
        std::ostringstream stdoutss;
        stdoutss << (success ? ACK : NCK) << std::endl;
        stdoutss << std::move(summery) << std::endl;
        std::cout << std::move(stdoutss.str());

        std::ostringstream stderrss;
        stderrss << EOT << std::endl;
        std::cerr << std::move(stderrss.str());
    }

    std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options)
    {
        std::promise<int> answer;

        std::ostringstream oss;
        oss << BEL << std::endl << question << GS;
        for (std::string const& option : options)
        {
            oss << option << GS;
        }

        oss << EOT << GS;

        std::cout << std::move(oss.str());

        std::string line;
        int index = -1;

        if (std::getline(std::cin, line))
        {
            try
            {
                index = std::stoi(line);
            }
            catch (std::exception const& e)
            {
            }
        }

        answer.set_value(index);
        return answer.get_future().share();
    }

  private:
    std::string name_;
    std::vector<std::string> parameters_;
};

class PseudoTest : public Test
{
  public:
    PseudoTest(std::string&& name, std::string&& plugin) : name_(std::move(name)), plugin_(std::move(plugin))
    {
    }

    ~PseudoTest()
    {
    }

    std::string const& getName() const final
    {
        return name_;
    }
    std::string const& getPlugin() const final
    {
        return plugin_;
    }

    std::shared_ptr<Action> nextAction() final
    {
        std::string line;
        std::vector<std::string> parameters;
        while (std::getline(std::cin, line))
        {
            if (line[0] == mil_preflight::EOT)
            {
                return nullptr;
            }
            else if (line[0] == mil_preflight::GS)
            {
                std::shared_ptr<mil_preflight::PseudoAction> action = std::make_shared<mil_preflight::PseudoAction>(
                    std::move(parameters[0]), std::vector(parameters.begin() + 1, parameters.end()));

                return action;
            }
            else
            {
                parameters.push_back(std::move(line));
            }
        }

        return nullptr;
    }

    void onFinish() final
    {
    }

  private:
    std::string name_;
    std::string plugin_;
};

}  // namespace mil_preflight

using Creator = std::shared_ptr<mil_preflight::PluginBase>();
boost::function<Creator> load(std::string const& plugin_name)
{
    try
    {
        return boost::dll::import_alias<Creator>(plugin_name, plugin_name,
                                                 boost::dll::load_mode::append_decorations |
                                                     boost::dll::load_mode::search_system_folders);
    }
    catch (boost::system::system_error const& e)
    {
        return []() -> std::shared_ptr<mil_preflight::PluginBase>
        { return std::make_shared<mil_preflight::PluginBase>(); };
    }
}

void runTest(std::shared_ptr<mil_preflight::Test> test, std::shared_ptr<mil_preflight::PluginBase> plugin)
{
    std::shared_ptr<mil_preflight::Action> action = test->nextAction();

    while (action != nullptr)
    {
        action->onStart();
        auto&& [success, summery] = plugin->runAction(action);
        action->onFinish(success, std::move(summery));
        action = test->nextAction();
    }

    test->onFinish();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    std::unordered_map<std::string, boost::function<Creator>> libraries;

    std::thread io_thread(
        [&]
        {
            std::string line;
            std::shared_ptr<mil_preflight::PluginBase> node;
            std::vector<std::string> parameters;
            while (std::getline(std::cin, line))
            {
                if (line[0] == mil_preflight::EOT)
                {
                    rclcpp::shutdown();
                    break;
                }
                else if (line[0] == mil_preflight::GS)
                {
                    auto it = libraries.find(parameters[1]);
                    if (it == libraries.end())
                    {
                        std::function<Creator> creator = load(parameters[1]);
                        it = libraries.emplace(parameters[1], std::move(creator)).first;
                    }

                    std::shared_ptr<mil_preflight::PseudoTest> test =
                        std::make_shared<mil_preflight::PseudoTest>(std::move(parameters[0]), std::move(parameters[1]));

                    node = it->second();
                    exec.add_node(node);
                    runTest(test, node);
                    exec.remove_node(node);
                    node.reset();

                    parameters.clear();
                }
                else
                {
                    parameters.emplace_back(std::move(line));
                }
            }
        });

    exec.spin();
    rclcpp::shutdown();
    io_thread.join();
    return 0;
}
