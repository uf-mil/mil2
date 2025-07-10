#include "mil_preflight/ui.h"

#include <boost/dll/alias.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

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

    void add_page(std::string const menu_entry, Component page, std::string const& title)
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

    void add_callback(std::string const menu_entry, std::function<void(void)> action)
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

    void initialize(int argc, char* argv[]) final
    {
        std::string filename;
        if (argc > 1)
        {
            filename = argv[1];
        }
        else
        {
            boost::filesystem::path filepath = boost::process::search_path("mil_preflight");
            filename = (filepath.parent_path() / "config.json").string();
        }

        root = std::make_shared<Root>();

        Component tests_page = std::make_shared<mil_preflight::TestsPage>(filename) | flex;
        Component reports_page = std::make_shared<mil_preflight::ReportsPage>() | flex;

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

        root->add_page("Run Tests", tests_page, "Tests");
        root->add_page("View Reports", reports_page, "Reports");
        root->add_page("About", about_page, "About");
        root->add_callback("Quit", [this] { screen.Exit(); });
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
};

}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::FTXUI::create, ftx_ui);
