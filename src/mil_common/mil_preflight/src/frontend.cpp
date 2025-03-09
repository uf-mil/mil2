#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

#include <vector>
#include <fstream>
#include <filesystem>
#include <unordered_map>

#include <boost/json.hpp>

#include "mil_preflight/uis/ftxui/testPage.h"
#include "mil_preflight/uis/ftxui/reportPage.h"

using namespace ftxui;

auto screen = ScreenInteractive::Fullscreen();

int main(int argc, char* argv[]) 
{
    // Get the configuration file name from cmd line args
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

    /*--------------------The page head-----------------------*/
    int currentPage = 0;
    bool showBackButton = false;
    std::string titles[] = {"MIL PreFlight", "Tests", "Report", "Documentation"};
    auto backButton = Button("<", [&] { currentPage = 0; showBackButton = false; }, ButtonOption::Ascii()) | Maybe(&showBackButton);
    auto head = Renderer(backButton, [&] {
        if(currentPage != 0)
            showBackButton = true;

        return vbox({
                hbox({backButton->Render(), text(titles[currentPage]) | bold | center | flex}),  // Back button & title in same line
                separator(),
            });
        });
    /*--------------------The main Tab-----------------------*/
    auto introduction = Renderer([] {
        return paragraph("Welcome to the Preflight Program, "
                "a tool inspired by the preflight checklists used by pilots before flying a plane. "
                "This program is designed to verify the functionality of all software and hardware systems on your autonomous robot. " 
                "It ensures that everything is in working order, " 
                "allowing you to safely deploy your robot with confidence.");
    });

    std::vector<std::string> menuItems = {
        "Run Tests", 
        "View Report", 
        "Read Documentation",
        "Quit"
    };

    int selectedIndex = 0;
    MenuOption menuOption = MenuOption::Vertical();
    menuOption.on_enter = [&]() 
    {
        if (selectedIndex == 3)
        {
            screen.Exit();
        }
        else
        {
            currentPage = selectedIndex + 1;
        }
    };
    auto menu = Menu(&menuItems, &selectedIndex, menuOption);
    // Layout using vbox (vertical layout)
    auto mainLayout = Container::Vertical({introduction, menu});

    auto mainTab = Renderer(mainLayout, [&] {
        return vbox({
            introduction->Render(),
            separator(),
            menu->Render() | center,
        }) | flex;
    });

    /*--------------------The test Tab-----------------------*/
    Component jobPage = std::make_shared<mil_preflight::TestPage>(fileName) | flex;

    /*--------------------The report Tab-----------------------*/
    // auto reportTab = Renderer([&] {
    //     return vbox({
    //         text("This is the report.") | center
    //     }) | flex;
    // });

    Component reportPage = std::make_shared<mil_preflight::ReportPage>() | flex;

    /*--------------------The documentation Tab-----------------------*/

    auto docTab = Renderer([&] {
        return vbox({
            text("Coming soon") | center
        }) | flex;
    });

    auto body = Container::Tab({
        mainTab,
        jobPage,
        reportPage,
        docTab
    }, &currentPage);

    auto page = Container::Vertical({head, body});

    screen.Loop(page);
    return 0;
}