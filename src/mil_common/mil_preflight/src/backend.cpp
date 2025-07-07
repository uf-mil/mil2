#include <filesystem>

#include <boost/dll.hpp>
#include <boost/function.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mil_preflight/common.h"
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
            if (line[0] == EOT)
            {
                return nullptr;
            }
            else if (line[0] == GS)
            {
                std::shared_ptr<PseudoAction> action = std::make_shared<PseudoAction>(
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

class Backend : public rclcpp::executors::SingleThreadedExecutor
{
  public:
    using Creator = std::shared_ptr<PluginBase>();

    Backend() : work_thread_([this] { run(); })
    {
    }

    ~Backend()
    {
        work_thread_.join();
    }

  private:
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
            return []() -> std::shared_ptr<PluginBase> { return std::make_shared<PluginBase>(); };
        }
    }

    std::unordered_map<std::string, boost::function<Creator>> libraries_;
    std::thread work_thread_;

    void run()
    {
        std::string line;
        std::shared_ptr<PluginBase> plugin;
        std::vector<std::string> parameters;
        while (std::getline(std::cin, line))
        {
            if (line[0] == EOT)
            {
                rclcpp::shutdown();
                break;
            }
            else if (line[0] == GS)
            {
                auto it = libraries_.find(parameters[1]);
                if (it == libraries_.end())
                {
                    std::function<Creator> creator = load(parameters[1]);
                    it = libraries_.emplace(parameters[1], std::move(creator)).first;
                }

                std::shared_ptr<PseudoTest> test =
                    std::make_shared<PseudoTest>(std::move(parameters[0]), std::move(parameters[1]));

                plugin = it->second();
                add_node(plugin);
                plugin->runTest(test);
                remove_node(plugin);
                plugin.reset();

                parameters.clear();
            }
            else
            {
                parameters.emplace_back(std::move(line));
            }
        }
    }
};

}  // namespace mil_preflight

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<mil_preflight::Backend> backend = std::make_shared<mil_preflight::Backend>();

    backend->spin();
    rclcpp::shutdown();

    return 0;
}
