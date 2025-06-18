#pragma once

#include <iostream>
#include <sstream>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

namespace mil_preflight
{
using namespace ftxui;

class Window : public ComponentBase, public std::enable_shared_from_this<Window>
{
  public:
    struct Option
    {
        std::string title;
    };

    Window(std::string const& title) : screen_(ScreenInteractive::Fullscreen())
    {
        Component close_button = Button("X", [=] { screen_.Exit(); }, ButtonOption::Ascii());

        head_ = Container::Horizontal({ Renderer([=] { return text(title); }) | center | flex, close_button });
    }

    ~Window()
    {
    }

    void show(Component content)
    {
        Component main = Container::Vertical({ head_, Renderer([] { return separator(); }), content | flex });
        Add(main | border);

        screen_.Loop(shared_from_this());
    }

    void close()
    {
        screen_.Exit();
    }

  protected:
  private:
    Component head_;
    ScreenInteractive screen_;
};

class Dialog : public Window
{
  public:
    Dialog(std::string const& title) : Window(title)
    {
    }

    ~Dialog()
    {
    }

    int show(Component content, std::vector<std::string> const& options)
    {
        int ret = -1;
        Components buttons;
        for (size_t i = 0; i < options.size(); i++)
        {
            buttons.push_back(Button(
                options[i],
                [this, &ret, i]
                {
                    ret = i;
                    close();
                },
                ButtonOption::Border()));
        }

        Component main = Container::Vertical(
            { content | flex, Renderer([] { return separator(); }), Container::Horizontal(buttons) | center });

        Window::show(main);
        return ret;
    }

  private:
};

class MessageBox : public Dialog
{
  public:
    MessageBox(std::string const& title) : Dialog(title)
    {
    }

    ~MessageBox()
    {
    }

    int show(std::string const& message, std::vector<std::string> const& options)
    {
        std::istringstream iss(message);
        std::string line;
        Elements paragraphs;
        while (std::getline(iss, line))
        {
            paragraphs.push_back(paragraph(line));
        }

        return Dialog::show(Renderer([&] { return vbox({ paragraphs }); }), options);
    }

  private:
    Option option_;
};

}  // namespace mil_preflight
