#pragma once
// Show a dialog on the terminal powered by ftxui

#include "ftxui/component/screen_interactive.hpp"
#include "ftxui/component/component.hpp"
#include "ftxui/dom/elements.hpp"

namespace mil_tools::tui
{
class Dialog
{
public:
    Dialog()
    {

    }

    int exec(const std::string& title, 
        ftxui::Component content,
        const std::vector<std::string>& buttons = {"OK"})
    {
        using namespace ftxui;
        Component close_button = Button("X", [&]{
            screen.Exit();
        }, ButtonOption::Ascii());

        Components button_comps;
        button_comps.push_back(Renderer([]{return filler();}));
        for(size_t i=0;i<buttons.size();i++)
        {
            button_comps.push_back(Button(buttons[i], [this, i=i]{
                    exit_code = i;
                    screen.Exit();
                },
                ButtonOption::Border()
            ));
        }

        Component dialog_ui = Container::Vertical({
            Renderer(close_button,[=]{
                return hbox({
                    text(title) | center | flex,
                    close_button->Render()
                });
            }),
            Renderer([]{return separator();}),
            content | vcenter | flex,
            Renderer([]{return separator();}),
            Container::Horizontal(button_comps)
        }) | border;
        
        screen.Loop(dialog_ui);
        return exit_code;
    }

    void close()
    {
        screen.Exit();
    }

    private:
    int exit_code = -1;
    ftxui::ScreenInteractive screen = ftxui::ScreenInteractive::Fullscreen();
};
}