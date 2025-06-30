#include "mil_preflight/ui.h"

#include <future>

#include <boost/dll/alias.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

#include "mil_preflight/uis/ftxui/dialog.h"
#include "mil_preflight/uis/ftxui/reportsPage.h"
#include "mil_preflight/uis/ftxui/testsPage.h"

using namespace ftxui;
auto screen = ScreenInteractive::Fullscreen();

namespace mil_preflight
{
class Root : public ComponentBase
{
  public:
    Root()
    {
        Component back_button = Button(
                                    "<", [this] { current_page = 0; }, ButtonOption::Ascii()) |
                                Maybe([this] { return current_page != 0; });
        Component head =
            Renderer(back_button,
                     [this, back_button]
                     {
                         return hbox({ back_button->Render(), text(titles[current_page]) | bold | center |
                                                                  flex });  // Back button & title in same line
                     });

        body = Container::Tab({}, &current_page);
        menu = Container::Vertical({});
        body->Add(menu | center);
        Add(Container::Vertical({ head, Renderer([] { return separator(); }), body | flex }));
    }
    ~Root()
    {
    }

    void addPage(std::string const menu_entry, Component page, std::string const& title)
    {
        ButtonOption menu_option = ButtonOption::Ascii();
        menu_option.transform = [](EntryState const& s)
        {
            std::string const t = s.active ? "> " + s.label + " ⋯"  //
                                             :
                                             "  " + s.label + " ⋯";
            return text(t) | (s.focused ? bold : nothing);
        };

        int index = body->ChildCount();
        menu->Add(Button(menu_entry, [this, index] { current_page = index; }, menu_option));
        body->Add(page);
        titles.push_back(title);
    }

    void addCallback(std::string const menu_entry, std::function<void(void)> action)
    {
        ButtonOption menu_option = ButtonOption::Ascii();
        menu_option.transform = [](EntryState const& s)
        {
            std::string const t = s.active ? "> " + s.label  //
                                             :
                                             "  " + s.label;
            return text(t) | (s.focused ? bold : nothing);
        };

        menu->Add(Button(menu_entry, action, menu_option));
    }

  private:
    int current_page = 0;
    std::vector<std::string> titles = { "MIL Preflight" };
    Component body;
    Component menu;
};

class FTXUI : public UIBase
{
  public:
    FTXUI()
    {
    }

    bool initialize(int argc, char* argv[]) final
    {
        std::optional<boost::property_tree::ptree> cfg = loadConfig(argc, argv);
        if (!cfg)
            return false;

        root = std::make_shared<Root>();

        auto tests_page = std::make_shared<mil_preflight::TestsPage>([this](TestsPage& page) { runJobAsync(page); });

        createJob(cfg.value(), tests_page);

        Component reports_page = std::make_shared<mil_preflight::ReportsPage>();

        Component about_page = Renderer(
            [this]
            {
                return vbox({ text("MIL Preflight") | bold, text("v0.1.3"), text("University of Florida"),
                              hbox({ text("Machine Intelligence Laboratory") | hyperlink("https://mil.ufl.edu/") }),
                              hbox({ text("Powered by "), text("FTXUI") | hyperlink("https://github.com/ArthurSonzogni/"
                                                                                    "FTXUI/") }),
                              separatorEmpty(),
                              paragraph("MIL Preflight is a tool inspired by the preflight checklists used by "
                                        "pilots before flying a plane."),
                              paragraph("This program is designed to verify the functionality of all software and "
                                        "hardware systems on your autonomous robot."),
                              paragraph("It ensures that everything is in working order, allowing you to safely "
                                        "deploy your robot with confidence.") }) |
                       flex;
            });

        root->addPage("Run Tests", tests_page, "Tests");
        root->addPage("View Reports", reports_page, "Reports");
        root->addPage("About", about_page, "About");
        root->addCallback("Quit", [this] { screen.Exit(); });

        return true;
    }

    ~FTXUI() final
    {
    }

    int spin() final
    {
        screen.Loop(root);
        return 0;
    }

    static std::shared_ptr<FTXUI> create()
    {
        return std::make_shared<FTXUI>();
    }

  private:
    std::shared_ptr<Root> root;

    std::optional<boost::property_tree::ptree> loadConfig(int argc, char* argv[])
    {
        boost::filesystem::path filename;
        if (argc > 1)
        {
            filename = argv[1];
        }
        else
        {
            filename = boost::process::search_path("mil_preflight").parent_path();
            std::vector<std::string> filenames;
            int selected = 0;

            // Create the RadioBox component
            for (auto const& entry : boost::filesystem::directory_iterator(filename))
            {
                if (entry.is_regular_file() && entry.path().extension() == ".json")
                {
                    filenames.push_back(entry.path().string());
                }
            }

            if (filenames.size() > 0)
            {
                auto content = Radiobox(&filenames, &selected);
                std::shared_ptr<Dialog> dialog = std::make_shared<Dialog>("Load config");
                if (dialog->show(content, { "Go" }) == -1)
                    return std::nullopt;
            }
            else
            {
                std::shared_ptr<MessageBox> message_box = std::make_shared<MessageBox>("Load config");
                message_box->show("No config file found under\n" + filename.string(), { "Quit" });
                return std::nullopt;
            }
            filename = filenames[selected];
        }

        // Parse the configuration file
        boost::property_tree::ptree cfg;
        try
        {
            boost::property_tree::read_json(filename.string(), cfg);
        }
        catch (boost::property_tree::json_parser_error const& e)
        {
            std::shared_ptr<MessageBox> message_box = std::make_shared<MessageBox>("Load config");
            message_box->show("Failed to parse\n" + std::string(e.what()), { "Quit" });
        }

        return cfg;
    }

    void createJob(boost::property_tree::ptree& cfg, std::shared_ptr<TestsPage> tests_page)
    {
        for (auto& test_pair : cfg)
        {
            std::string test_name = std::move(test_pair.first);
            boost::property_tree::ptree& test_node = test_pair.second;

            std::shared_ptr<TestTab> test_tab =
                tests_page->createTest(std::move(test_name), test_node.get<std::string>("plugin"));

            if (!test_tab)
                continue;

            for (auto& action_pair : test_node.get_child("actions"))
            {
                std::string action_name = std::move(action_pair.first);
                boost::property_tree::ptree& params_array = action_pair.second;

                std::vector<std::string> parameters;
                for (auto& param : params_array)
                {
                    parameters.push_back(std::move(param.second.get_value<std::string>()));
                }

                test_tab->createAction(std::move(action_name), std::move(parameters));
            }
        }
    }

    std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options) final
    {
        std::shared_ptr<std::promise<int>> feedback = std::make_shared<std::promise<int>>();

        std::shared_ptr<MessageBox> dialog = std::make_shared<MessageBox>("Question");
        screen.Post(
            [=]
            {
                int index = dialog->show(question, options);
                feedback->set_value(index);
            });

        return feedback->get_future().share();
    }
};

}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::FTXUI::create, ftx_ui);
