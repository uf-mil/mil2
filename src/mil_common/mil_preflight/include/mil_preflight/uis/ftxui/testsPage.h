#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include "mil_preflight/ui.h"

namespace mil_preflight
{
using namespace ftxui;

struct ActionBoxOption
{
    std::string name;
    std::vector<std::string> parameters;
    std::function<void(bool)> onChange;
    std::function<bool(bool)> transform;
};

class ActionBox : public ComponentBase, public Action
{
  public:
    ActionBox(ActionBoxOption&& option);
    ~ActionBox();

    bool isChecked() const
    {
        return option_.transform(checked_);
    }
    void check()
    {
        checked_ = true;
    }
    void uncheck()
    {
        checked_ = false;
    }
    void reset()
    {
        state_ = State::NONE;
    }

    std::string const& getName() const final
    {
        return option_.name;
    }
    std::vector<std::string> const& getParameters() const final
    {
        return option_.parameters;
    }

  private:
    enum class State
    {
        NONE,
        RUNNING,
        SUCCESS,
        FAILED
    };

    bool checked_ = false;
    bool hovered_ = false;
    Box box_;
    std::atomic<State> state_ = State::NONE;
    ActionBoxOption option_;

    Element Render() final;
    bool OnEvent(Event event) final;
    inline bool OnMouseEvent(Event event);
    bool Focusable() const final;

    void onStart() final;
    void onFinish(bool success, std::string&& summery) final;
    std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options) final;
};

struct TestTabOption
{
    std::string name;
    std::string plugin;
    std::function<void(bool)> onChange;
    std::function<bool(bool)> transform;
    Component childContainer;
};

class TestTab : public ComponentBase, public Test
{
  public:
    TestTab(TestTabOption&& option) : option_(std::move(option))
    {
    }
    ~TestTab()
    {
    }

    bool isChecked()
    {
        return option_.transform(checked_) || nChecked_ > 0;
    }

    bool transform(bool checked)
    {
        return checked || option_.transform(checked_);
    }

    virtual std::string const& getPlugin() const
    {
        return option_.plugin;
    }
    virtual std::string const& getName() const
    {
        return option_.name;
    }

    std::shared_ptr<ActionBox> createAction(std::string&& name, std::vector<std::string>&& parameters);

    std::shared_ptr<Action> nextAction() final;
    void onFinish() final;

  private:
    bool hovered_ = false;
    bool toggle_ = false;

    size_t nChecked_ = 0;
    size_t currentAction_ = 0;
    bool checked_ = false;

    TestTabOption option_;

    Box box_;

    Element Render() final;
    bool OnEvent(Event event) final;
    bool OnMouseEvent(Event event);

    bool Focusable() const final
    {
        return true;
    }
};

class TestsPage : public ComponentBase, public Job, public std::enable_shared_from_this<TestsPage>
{
  public:
    TestsPage(std::function<void(std::shared_ptr<TestsPage>)> onRun);
    ~TestsPage();
    std::shared_ptr<TestTab> createTest(std::string&& name, std::string&& plugin);

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

    std::shared_ptr<Test> nextTest() final;
    void onFinish() final;
    bool OnEvent(Event event) final;
};

}  // namespace mil_preflight
