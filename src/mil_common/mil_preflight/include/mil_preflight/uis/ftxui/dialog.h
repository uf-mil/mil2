#pragma once

#include <iostream>
#include <sstream>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

namespace mil_preflight
{
using namespace ftxui;

class Dialog : public ComponentBase, public std::enable_shared_from_this<Dialog>
{
  public:
    struct Option
    {
        std::string title;
        std::string question;
        std::vector<std::string> buttonLabels;
    };

    Dialog(Option const& option) : option_(option), screen_(ScreenInteractive::Fullscreen())
    {
        create();
    }

    Dialog(Option&& option) : option_(std::move(option)), screen_(ScreenInteractive::Fullscreen())
    {
        create();
    }

    ~Dialog()
    {
    }

    int show()
    {
        screen_.Loop(shared_from_this());
        return index_;
    }

    Element Render() override
    {
        return vbox({ hbox({ text(option_.title) | flex, closeButton_->Render() }), separator(),
                      vbox(paragraphs_) | flex, separator(), buttonsContainer_->Render() | center }) |
               border;
    }

  private:
    Component buttonsContainer_;
    Component closeButton_;
    Elements paragraphs_;
    Option option_;
    ScreenInteractive screen_;
    int index_ = -1;

    void create()
    {
        std::istringstream iss(option_.question);
        std::string line;
        while (std::getline(iss, line))
        {
            paragraphs_.push_back(paragraph(line));
        }

        Components buttons;
        for (size_t i = 0; i < option_.buttonLabels.size(); i++)
        {
            buttons.push_back(Button(
                option_.buttonLabels[i],
                [=]
                {
                    index_ = i;
                    screen_.Exit();
                },
                ButtonOption::Border()));
        }
        buttonsContainer_ = Container::Horizontal(buttons);

        closeButton_ = Button(
            "X",
            [=]
            {
                index_ = -1;
                screen_.Exit();
            },
            ButtonOption::Ascii());

        Component dialogContainer = Container::Vertical({ closeButton_, buttonsContainer_ });

        Add(dialogContainer);
    }
};

}  // namespace mil_preflight
