#pragma once

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include "mil_preflight/ui.h"

namespace mil_preflight
{
using namespace ftxui;

using TestReport = std::unordered_map<std::string, Action::Report>;
using JobReport = std::unordered_map<std::string, TestReport>;

class ReportsPage : public ComponentBase
{
  public:
    ReportsPage();
    ~ReportsPage();

  private:
    JobReport report_;

    bool showSuccess_ = true;
    Component reportPanel_;
    Component bottom_;
    int selector_ = 0;

    Element Render() final;
    bool OnEvent(Event event) final;
};

}  // namespace mil_preflight
