#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include "mil_preflight/job.h"

namespace mil_preflight
{
using namespace ftxui;

class TestsPage : public ComponentBase, public Job
{
  public:
    TestsPage(std::string const& filePath);
    ~TestsPage();

  private:
    Component tabsContainer_;
    Component pagesContainer_;
    Component main_;

    int selector_ = 0;
    std::vector<int> actionSelectors_;
    bool selectAll_ = false;
    size_t nSelected_ = 0;
    int mainSize_ = 20;

    std::atomic<size_t> currentTest_ = 0;
    std::atomic<bool> running_ = false;

    std::string const buttonLabels_[4] = { " Run ", " ·   ", "  ·  ", "   · " };
    size_t ticker_ = 0;

    std::optional<std::reference_wrapper<Test>> nextTest() final;
    std::optional<std::reference_wrapper<Test>> createTest(std::string&& name, std::string&& plugin) final;
    void onFinish(Job::Report&& report) final;

    bool OnEvent(Event event) final;
};

}  // namespace mil_preflight
