#include "mil_preflight/ui.h"

#include <boost/dll/alias.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

#include "mil_preflight/uis/ftxui/reportsPage.h"
#include "mil_preflight/uis/ftxui/testsPage.h"

using namespace ftxui;
auto screen = ScreenInteractive::Fullscreen();

namespace mil_preflight
{

class FTXUI : public UIBase
{
  public:
    FTXUI()
    {
    }

    void initialize(int argc, char* argv[]) final
    {
        std::string fileName;
        if (argc > 1)
        {
            fileName = argv[1];
        }
        else
        {
            boost::filesystem::path filePath = boost::process::search_path("mil_preflight");
            fileName = (filePath.parent_path() / "config.json").string();
        }

        Component backButton = Button(
                                   "<",
                                   [=]
                                   {
                                       currentPage_ = 0;
                                       showBackButton_ = false;
                                   },
                                   ButtonOption::Ascii()) |
                               Maybe(&showBackButton_);
        Component head = Renderer(backButton,
                                  [=]
                                  {
                                      if (currentPage_ != 0)
                                          showBackButton_ = true;

                                      return vbox({
                                          hbox({ backButton->Render(), text(titles_[currentPage_]) | bold | center |
                                                                           flex }),  // Back button & title in same line
                                          separator(),
                                      });
                                  });

        Component menu = Container::Vertical({}, &selectedIndex_);

        ButtonOption menuOption = ButtonOption::Ascii();
        menuOption.transform = [](EntryState const& s)
        {
            std::string const t = s.active ? "> " + s.label  //
                                             :
                                             "  " + s.label;
            return text(t) | (s.focused ? bold : nothing);
        };

        menu->Add(Button("Run Tests", [=] { currentPage_ = 1; }, menuOption));
        menu->Add(Button("View Reports", [=] { currentPage_ = 2; }, menuOption));
        menu->Add(Button("Read Documentation", [=] { currentPage_ = 3; }, menuOption));
        menu->Add(Button("Quit", [&] { screen.Exit(); }, menuOption));

        Component mainPage = Renderer(menu,
                                      [=]
                                      {
                                          return vbox({
                                                     paragraph(introduction_),
                                                     separator(),
                                                     menu->Render() | center | flex | frame,
                                                 }) |
                                                 flex;
                                      });

        Component testsPage = std::make_shared<mil_preflight::TestsPage>(fileName) | flex;
        Component reportsPage = std::make_shared<mil_preflight::ReportsPage>() | flex;

        Component docPage = Renderer([] { return vbox({ text("Coming soon") | center }) | flex; });

        Component body = Container::Tab({ mainPage, testsPage, reportsPage, docPage }, &currentPage_);

        root_ = Container::Vertical({ head, body });
    }

    ~FTXUI() final
    {
    }

    int spin() final
    {
        screen.Loop(root_);
        return 0;
    }

    static std::shared_ptr<FTXUI> create()
    {
        return std::make_shared<FTXUI>();
    }

  private:
    int selectedIndex_ = 0;
    int currentPage_ = 0;
    bool showBackButton_ = false;
    Component root_;

    std::string const introduction_ = "Welcome to the Preflight Program, "
                                      "a tool inspired by the preflight checklists used by pilots before flying a "
                                      "plane. "
                                      "This program is designed to verify the functionality of all software and "
                                      "hardware "
                                      "systems on your autonomous robot. "
                                      "It ensures that everything is in working order, "
                                      "allowing you to safely deploy your robot with confidence.";
    std::vector<std::string> titles_ = { "MIL Preflight", "Tests", "Reports", "Documentation" };
};

}  // namespace mil_preflight

BOOST_DLL_ALIAS(mil_preflight::FTXUI::create, ftx_ui);
