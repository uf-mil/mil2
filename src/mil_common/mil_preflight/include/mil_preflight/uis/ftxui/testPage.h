#pragma once

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include <atomic>
#include <memory>
#include <string>

#include "mil_preflight/job.h"

namespace mil_preflight
{
using namespace ftxui;

class TestPanel;

class TestPage: public ComponentBase
{
  public:
  TestPage(std::string const& filePath);
  ~TestPage();

  private:
  const std::string buttonLabels_[4] = {" Run ", " ·   ", "  ·  ", "   · "};
  size_t ticker_ = 0;
  
  JobRunner runner_;
  std::shared_ptr<TestPanel> panel_;

  bool selectAll_ = false;
  int selector_ = 0;


  Component main_;

  bool OnEvent(Event event) final;
  // Element Render() final;
};
}