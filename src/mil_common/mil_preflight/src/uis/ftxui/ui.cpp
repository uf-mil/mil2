#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

#include <boost/dll/alias.hpp>

#include "mil_preflight/uis/ftxui/testPage.h"
#include "mil_preflight/uis/ftxui/reportPage.h"
#include "mil_preflight/ui.h"

using namespace ftxui;
auto screen = ScreenInteractive::Fullscreen();

namespace mil_preflight
{

class FTXUI: public UIBase
{
    public:
    FTXUI()
    {
        
    }

    void initialize(int argc, char* argv[]) final
    {
        std::string fileName;
        if(argc > 1)
        {
            fileName = argv[1];
        }
        else
        {
            auto binPath = std::filesystem::canonical("/proc/self/exe").parent_path();
            fileName = binPath / ".." / "cfg" / "config.json";
        }

        Component backButton = Button("<", [=] { 
            currentPage_ = 0; 
            showBackButton_ = false; 
        }, ButtonOption::Ascii()) | Maybe(&showBackButton_);
        Component head = Renderer(backButton, [=] {
            if(currentPage_ != 0)
                showBackButton_ = true;

            return vbox({
                    hbox({backButton->Render(), text(titles_[currentPage_]) | bold | center | flex}),  // Back button & title in same line
                    separator(),
                });
            });
        
        MenuOption menuOption = MenuOption::Vertical();
        menuOption.on_enter = [=]() 
        {
            if (selectedIndex_ == 3)
            {
                screen.Exit();
            }
            else
            {
                currentPage_ = selectedIndex_ + 1;
            }
        };

        Component menu = Menu(&menuItems_, &selectedIndex_, menuOption);

        Component mainPage = Renderer(menu, [=] {
            return vbox({
                paragraph(introduction_),
                separator(),
                menu->Render() | center,
            }) | flex;
        });

        Component testPage = std::make_shared<mil_preflight::TestPage>(fileName) | flex;
        Component reportsPage = std::make_shared<mil_preflight::ReportsPage>() | flex;
        
        Component docPage = Renderer([] {
            return vbox({
                text("Coming soon") | center
            });
        }) | flex;
    
        Component body = Container::Tab({
            mainPage,
            testPage,
            reportsPage,
            docPage
        }, &currentPage_);
    
        root_ = Container::Vertical({head, body});
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

    const std::string introduction_ = "Welcome to the Preflight Program, "
                    "a tool inspired by the preflight checklists used by pilots before flying a plane. "
                    "This program is designed to verify the functionality of all software and hardware systems on your autonomous robot. " 
                    "It ensures that everything is in working order, " 
                    "allowing you to safely deploy your robot with confidence.";
    std::vector<std::string> titles_ = {"MIL PreFlight", "Tests", "Report", "Documentation"};
    std::vector<std::string> menuItems_ = {"Run Tests", "View Report", "Read Documentation", "Quit"};
};

}

BOOST_DLL_ALIAS(mil_preflight::FTXUI::create, ftx_ui);