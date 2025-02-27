#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

#include <vector>
#include <fstream>
#include <filesystem>
#include <unordered_map>

#include <boost/json.hpp>

#include "mil_preflight/widgets.h"

using namespace ftxui;

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

    std::ifstream file(fileName);
    if(!file.is_open())
    {
        std::cout << "Failed to load the configuration file: "<<  fileName << std::endl;
        return -1;
    }

    // Parse the configuration file
    boost::json::value data = boost::json::parse(file);

    file.close();


    auto screen = ScreenInteractive::Fullscreen();

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
    auto menu = Menu(&menuItems, &selectedIndex);

    auto menuHandler = CatchEvent(menu, [&](Event event) {
        if (event == Event::Return) {  // Press Enter
            if (selectedIndex == 3)
            {
                screen.Exit();
            }
            else
            {
                currentPage = selectedIndex + 1;
            }
            return true;
        }
        return false; // Continue processing events
    });

    // Layout using vbox (vertical layout)
    auto mainLayout = Container::Vertical({introduction,menuHandler});

    auto mainTab = Renderer(mainLayout, [&] {
        return vbox({
            introduction->Render(),
            separator(),
            menu->Render() | center,
        }) | flex;
    });

    /*--------------------The test Tab-----------------------*/
    mil_preflight::JobWidget job(data);

    auto backButton1 = Button("<", [&] { currentPage = 0;}, ButtonOption::Ascii());

    auto testsTab = job.getCompoent() | flex;
    
    // auto testsPage = Container::Vertical({head1, Renderer([&]{return separator();}) ,jobContainer | flex});
    // auto testsPage = Renderer(pageContainer, [&] {
    //     return vbox({
    //         hbox({backButton1->Render(), text("Tests") | bold | center | flex}),  // Back button & title in same line
    //         separator(),
    //         jobContainer->Render() | flex,
    //         runButton->Render() | align_right
    //     });
    // });

    /*--------------------The report Tab-----------------------*/
    auto reportTab = Renderer([&] {
        return vbox({
            text("This is the report.") | center
        }) | flex;
    });

    /*--------------------The documentation Tab-----------------------*/

    auto docTab = Renderer([&] {
        return vbox({
            text("Coming soon") | center
        }) | flex;
    });

    auto body = Container::Tab({
        mainTab,
        testsTab,
        reportTab,
        docTab
    }, &currentPage);

    auto page = Container::Vertical({head, body});

    screen.Loop(page);
    return 0;
}