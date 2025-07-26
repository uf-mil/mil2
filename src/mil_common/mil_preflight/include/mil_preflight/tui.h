#pragma once

#include <ftxui/component/component.hpp>

#include "mil_preflight/frontend.h"

namespace mil_preflight
{

using namespace ftxui;

class TUI : public ComponentBase
{
  public:
    TUI(int argc, char* argv[]);
    ~TUI();
};

}  // namespace mil_preflight
