// Copyright 2020 Arthur Sonzogni. All rights reserved.
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.
#include <memory>  // for shared_ptr, __shared_ptr_access
#include <string>  // for string, basic_string, operator+, to_string
#include <vector>  // for vector
 
#include "ftxui/component/captured_mouse.hpp"      // for ftxui
#include "ftxui/component/component.hpp"           // for Radiobox, Renderer
#include "ftxui/component/component_base.hpp"      // for ComponentBase
#include "ftxui/component/screen_interactive.hpp"  // for ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for operator|, Element, size, border, frame, HEIGHT, LESS_THAN
 
using namespace ftxui;
 
int main() {
  bool* states = new bool[30];
  std::vector<std::string> labels;
  std::vector<Component> checkboxes;

  for (int i = 0; i < 30; ++i)
  {
    labels.emplace_back("CheckBox " + std::to_string(i));
    checkboxes.push_back(Checkbox(labels[i], &states[i]));
  }

  auto container = Container::Vertical(checkboxes);


  auto renderer = Renderer(container, [&] {
    return container -> Render() | vscroll_indicator | frame |
           size(HEIGHT, LESS_THAN, 10) | border;
  });

  
 
  auto screen = ScreenInteractive::FitComponent();
  screen.Loop(renderer);

  delete[] states;
 
  return 0;
}