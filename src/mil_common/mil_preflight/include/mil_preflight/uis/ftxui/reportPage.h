#pragma once

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include "mil_preflight/job.h"

namespace mil_preflight
{
using namespace ftxui;

class ReportPage: public ComponentBase
{
    public:
    ReportPage();
    ~ReportPage();

    private:
    Job::Report report_;

    bool showSuccess_ = true;
    Component summeryPanel_;

    Element Render() final;
};

}