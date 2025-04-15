#pragma once

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include "mil_preflight/job.h"

namespace mil_preflight
{
using namespace ftxui;

class ReportsPage : public ComponentBase
{
  public:
    ReportsPage();
    ~ReportsPage();

  private:
    Job::Report report_;

    bool showSuccess_ = true;
    Component reportPanel_;
    Component bottom_;
    int selector_ = 0;

    Element Render() final;
    bool OnEvent(Event event) final;
};

}  // namespace mil_preflight
