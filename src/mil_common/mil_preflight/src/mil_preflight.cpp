#include <future>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

#include "mil_preflight/frontend.h"
#include "mil_preflight/tui.h"

ftxui::ScreenInteractive screen = ftxui::ScreenInteractive::Fullscreen();

int main(int argc, char* argv[])
{
    std::shared_ptr<mil_preflight::Frontend> frontend = std::make_shared<mil_preflight::Frontend>(argc - 1, &argv[1]);
    std::shared_ptr<mil_preflight::TUI> tui = std::make_shared<mil_preflight::TUI>(frontend);

    screen.Loop(tui);
    return 0;
}
