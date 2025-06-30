#include <functional>
#include <queue>
#include <sstream>
#include <stack>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

#include "mil_preflight/uis/ftxui/dialog.h"
#include "mil_preflight/uis/ftxui/reportsPage.h"
#include "mil_preflight/uis/ftxui/testsPage.h"

using namespace ftxui;
extern ScreenInteractive screen;

namespace mil_preflight
{

static std::queue<Job::Report> reportQueue;

static std::queue<std::shared_ptr<Dialog>> questionQueue;

ActionBox::ActionBox(ActionBoxOption&& option) : option_(std::move(option))
{
}

ActionBox::~ActionBox()
{
}

Element ActionBox::Render()
{
    bool focused = Focused();
    // bool active = Active();

    char const* indicator;
    Color textColor = Color::White;
    switch (state_)
    {
        case State::RUNNING:
            indicator = " ▶ ";
            textColor = Color::White;
            break;
        case State::SUCCESS:
            indicator = " ✔ ";
            textColor = Color::Green;
            break;
        case State::FAILED:
            indicator = " ✘ ";
            textColor = Color::Red;
            break;
        default:
            indicator = "   ";
            textColor = Color::White;
            break;
    }

    auto labelEle = text(option_.name);

    if (focused || hovered_)
    {
        labelEle |= inverted;
    }

    if (focused)
    {
        labelEle |= bold;
    }

    bool checked = option_.transform(checked_);

    auto element = hbox(
        { text(checked ? "☑ " : "☐ "), labelEle, filler() | flex, text(indicator) | color(textColor) | align_right });

    return element | (focused ? focus : nothing) | reflect(box_);
}

bool ActionBox::OnEvent(Event event)
{
    if (!CaptureMouse(event))
    {
        return false;
    }

    if (event.is_mouse())
    {
        return OnMouseEvent(event);
    }

    hovered_ = false;
    if (event == Event::Character(' ') || event == Event::Return)
    {
        checked_ = !checked_;
        option_.onChange(checked_);
        TakeFocus();
        return true;
    }

    return false;
}

bool ActionBox::OnMouseEvent(Event event)
{
    hovered_ = box_.Contain(event.mouse().x, event.mouse().y);

    if (!CaptureMouse(event))
    {
        return false;
    }

    if (!hovered_)
    {
        return false;
    }

    if (event.mouse().button == Mouse::Left && event.mouse().motion == Mouse::Pressed)
    {
        if (Focused())
        {
            checked_ = !checked_;
            option_.onChange(checked_);
        }
        else
            TakeFocus();
        return true;
    }

    return false;
}

bool ActionBox::Focusable() const
{
    return true;
}

void ActionBox::onStart()
{
    state_ = State::RUNNING;
}

void ActionBox::onFinish(Action::Report const& report)
{
    state_ = report.success ? State::SUCCESS : State::FAILED;
    screen.PostEvent(Event::Character("ActionFinish"));
}

std::optional<std::reference_wrapper<Action>> TestTab::nextAction()
{
    Component child = option_.childContainer;
    while (currentAction_ < child->ChildCount())
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(currentAction_++));
        child->SetActiveChild(action);
        if (action->isChecked())
            return *action;

        action->reset();
    }

    return std::nullopt;
}

std::shared_ptr<ActionBox> TestTab::createAction(std::string&& name, std::vector<std::string>&& parameters)
{
    ActionBoxOption option;
    option.name = std::move(name);
    option.parameters = std::move(parameters);
    option.onChange = [&](bool checked)
    {
        if (checked)
            nChecked_++;
        else
            nChecked_--;
    };

    option.transform = [&](bool checked) { return checked || option_.transform(checked_); };

    std::shared_ptr<ActionBox> action = std::make_shared<ActionBox>(std::move(option));
    option_.childContainer->Add(action);
    return action;
}

void TestTab::onFinish([[maybe_unused]] Test::Report const& report)
{
    currentAction_ = 0;
    screen.PostEvent(Event::Character("TestFinish"));
}

Element TestTab::Render()
{
    bool focused = Focused();

    auto labelEle = text(option_.name);

    if (focused || hovered_)
    {
        labelEle |= inverted;
    }

    if (focused)
    {
        labelEle |= bold;
    }

    char const* prefix;
    bool checked = option_.transform(checked_);
    if (checked || nChecked_ == option_.childContainer->ChildCount())
    {
        prefix = "☑ ";
    }
    else if (nChecked_ == 0)
    {
        prefix = "☐ ";
    }
    else
    {
        prefix = "▣ ";
    }

    auto element = hbox({ text(prefix), labelEle });

    return element | (focused ? focus : nothing) | reflect(box_);
}

bool TestTab::OnEvent(Event event)
{
    if (!CaptureMouse(event))
    {
        return false;
    }

    if (event.is_mouse())
    {
        return OnMouseEvent(event);
    }

    hovered_ = false;
    if (event == Event::Character(' ') || event == Event::Return)
    {
        checked_ = !checked_;
        option_.onChange(checked_);
        TakeFocus();
        return true;
    }

    return false;
}

bool TestTab::OnMouseEvent(Event event)
{
    hovered_ = box_.Contain(event.mouse().x, event.mouse().y);

    if (!CaptureMouse(event))
    {
        return false;
    }

    if (!hovered_)
    {
        return false;
    }

    if (event.mouse().button == Mouse::Left && event.mouse().motion == Mouse::Pressed)
    {
        if (Focused())
        {
            checked_ = !checked_;
            option_.onChange(checked_);
        }
        else
            TakeFocus();
        return true;
    }

    return false;
}

// class ActionList : public ComponentBase
// {
//   public:
//     using History = std::pair<size_t, bool>;

//     ActionList(std::shared_ptr<TestTab> tab);
//     ~ActionList();

//   private:
//     int selector_ = 0;
//     Box box_;
// };

// ActionList::ActionList(std::shared_ptr<TestTab> tab)
// {
//     Components comps;
//     Add(Container::Vertical(comps, &selector_));
// }

// ActionList::~ActionList()
// {

// }

TestsPage::TestsPage(std::function<void(TestsPage& page)> onRun)
{
    Components pages;
    Components tabs;

    tabsContainer_ = Container::Vertical(tabs, &selector_);
    pagesContainer_ = Container::Tab(pages, &selector_);
    main_ = ResizableSplitLeft(tabsContainer_ | vscroll_indicator | frame, pagesContainer_ | vscroll_indicator | frame,
                               &mainSize_);

    ButtonOption buttonOption = ButtonOption::Simple();
    buttonOption.transform = [&](EntryState const& s)
    {
        auto element = (running_ ? text(buttonLabels_[ticker_ % 3 + 1]) : text(s.label)) | border;
        if (s.active)
        {
            element |= bold;
        }
        if (s.focused)
        {
            element |= inverted;
        }
        return element;
    };

    Component runButton = Button(
        buttonLabels_[0],
        [=]
        {
            main_->TakeFocus();
            onRun(*this);
        },
        buttonOption);

    CheckboxOption option = CheckboxOption::Simple();
    option.transform = [&](EntryState const& s)
    {
        char const* prefix;

        if (s.state || nSelected_ == pagesContainer_->ChildCount())
        {
            prefix = "☑ ";
        }
        else if (nSelected_ == 0)
        {
            prefix = "☐ ";
        }
        else
        {
            prefix = "▣ ";
        }

        auto t = text(s.label);
        if (s.active)
        {
            t |= bold;
        }
        if (s.focused)
        {
            t |= inverted;
        }
        return hbox({ text(prefix), t });
    };
    Component checkBox = Checkbox("Select all", &selectAll_, option);
    Component bottom = Container::Horizontal({ checkBox | vcenter | flex, runButton });
    Add(Container::Vertical({ main_ | flex, Renderer([] { return separator(); }), bottom }));
}

TestsPage::~TestsPage()
{
}

std::optional<std::reference_wrapper<Test>> TestsPage::nextTest()
{
    running_ = true;

    while (currentTest_ < tabsContainer_->ChildCount())
    {
        std::shared_ptr<TestTab> test = std::dynamic_pointer_cast<TestTab>(tabsContainer_->ChildAt(currentTest_++));
        if (test->isChecked())
        {
            tabsContainer_->SetActiveChild(tabsContainer_->ChildAt(currentTest_ - 1));
            return *test;
        }
    }
    return std::nullopt;
}

std::shared_ptr<TestTab> TestsPage::createTest(std::string&& name, std::string&& plugin)
{
    actionSelectors_.push_back(0);
    Component list = Container::Vertical({}, &actionSelectors_.back());

    TestTabOption option;
    option.name = std::move(name);
    option.plugin = std::move(plugin);
    option.transform = [&](bool checked) { return checked || selectAll_; };
    option.onChange = [&](bool checked)
    {
        if (checked)
            nSelected_++;
        else
            nSelected_--;
    };
    option.childContainer = list;

    std::shared_ptr<TestTab> tab = std::make_shared<TestTab>(std::move(option));

    tabsContainer_->Add(tab);
    pagesContainer_->Add(list);

    return tab;
}

void TestsPage::onFinish(Job::Report&& report)
{
    running_ = false;
    currentTest_ = 0;
    reportQueue.push(std::move(report));
    screen.PostEvent(Event::Character("JobFinish"));
}

bool TestsPage::OnEvent(Event event)
{
    if (event == Event::Character("JobFinish"))
    {
        return false;
    }

    if (event == Event::Character("TestFinish"))
        return false;

    if (event == Event::Character("ActionFinish"))
    {
        ticker_++;
        return false;
    }

    if (running_ && main_->Focused())
        return false;

    return ComponentBase::OnEvent(event);
}

class ActionReportPanel : public ComponentBase
{
  public:
    ActionReportPanel(Action::Report&& report) : report_(std::move(report))
    {
        for (std::string const& line : report_.stdouts)
        {
            stdouts_.push_back(paragraph(line));
        }

        for (std::string const& line : report_.stderrs)
        {
            stderrs_.push_back(paragraph(line));
        }

        Component stdoutsRenderer = Renderer([=] { return vbox(stdouts_); });

        Component stdoutsCollap = Collapsible("stdout", stdoutsRenderer);

        Component stderrsRenderer = Renderer([=] { return vbox(stderrs_); });

        Component stderrsCollap = Collapsible("stderr", stderrsRenderer);

        Add(Container::Vertical(
            { Renderer([&] { return paragraph(report_.summery); }), stdoutsCollap, stderrsCollap }));
    }
    ~ActionReportPanel()
    {
    }

  private:
    Action::Report&& report_;
    Elements summeries_;
    Elements stdouts_;
    Elements stderrs_;
};

class TestReportPanel : public ComponentBase
{
  public:
    TestReportPanel(std::string const& name, Test::Report&& report, bool* errorOnly) : errorOnly_(errorOnly)
    {
        Component panelsContainer = Container::Tab({}, &selector_);
        Component tabsContainer = Container::Vertical({}, &selector_);
        int errorCount = 0;
        for (auto& pair : report)
        {
            ButtonOption option = ButtonOption::Simple();
            option.transform = option.transform = [success = pair.second.success](EntryState const& s)
            {
                Element element = text(s.label) | color(success ? Color::Green : Color::Red);
                if (s.focused)
                    element |= inverted;
                if (s.active)
                    element |= bold;
                return element;
            };

            Component button = Button(pair.first, [] {}, option);
            if (pair.second.success)
                button = Maybe(button, [=] { return !(*errorOnly_); });
            tabsContainer->Add(button);
            names_.push_back(pair.first);
            panelsContainer->Add(std::make_shared<ActionReportPanel>(std::move(pair.second)));

            if (!pair.second.success)
                errorCount++;
        }

        tab_ = Collapsible(name, Renderer(tabsContainer, [=] { return hbox({ text("  "), tabsContainer->Render() }); }),
                           &show_);

        if (errorCount == 0)
            tab_ = Maybe(tab_, [=] { return !(*errorOnly_); });

        Add(panelsContainer);
    }
    ~TestReportPanel()
    {
    }

    Component getTab()
    {
        return tab_;
    }

    bool isShown()
    {
        return show_;
    }

  private:
    int selector_ = 0;
    bool show_ = false;
    std::vector<std::string> names_;
    Component tab_;
    bool* errorOnly_;
};

class JobReportPanel : public ComponentBase
{
  public:
    JobReportPanel(Job::Report&& report, bool* errorOnly) : report_(std::move(report)), errorOnly_(errorOnly)
    {
    }

    ~JobReportPanel()
    {
    }

    Element Render() final
    {
        if (!rendered_)
        {
            left_ = Container::Vertical({}, &selector_);
            right_ = Container::Tab({}, &selector_);
            for (auto& pair : report_)
            {
                std::shared_ptr<TestReportPanel> panel =
                    std::make_shared<TestReportPanel>(pair.first, std::move(pair.second), errorOnly_);
                left_->Add(panel->getTab());
                right_->Add(panel);
            }

            Component maybe = Maybe(right_,
                                    [=]
                                    {
                                        auto panel =
                                            std::dynamic_pointer_cast<TestReportPanel>(right_->ChildAt(selector_));
                                        return panel->isShown();
                                    });

            Add(ResizableSplitLeft(left_ | vscroll_indicator | frame, maybe | flex | vscroll_indicator | yframe,
                                   &mainSize_));

            rendered_ = true;
        }

        return ChildAt(0)->Render();
    }

  private:
    Job::Report report_;
    Component left_;
    Component right_;
    int selector_ = 0;
    bool rendered_ = false;
    bool* errorOnly_;
    int mainSize_ = 20;
};

ReportsPage::ReportsPage()
{
    reportPanel_ = Container::Tab({}, &selector_);
    Component clearButton = Button(
        "Delete",
        [=]
        {
            if (reportPanel_->ChildCount() > 0)
            {
                reportPanel_->ChildAt(selector_)->Detach();
                selector_ = std::max(selector_ - 1, 0);
            }
        },
        ButtonOption::Border());
    Component bottomMiddle = Container::Horizontal({
        Button(
            "<", [=] { selector_ = std::min(selector_ + 1, static_cast<int>(reportPanel_->ChildCount() - 1)); },
            ButtonOption::Ascii()) |
            vcenter,
        Renderer(
            [=]
            {
                return text(std::to_string(reportPanel_->ChildCount() - selector_) + "/" +
                            std::to_string(reportPanel_->ChildCount()));
            }) |
            vcenter,
        Button(
            ">", [=] { selector_ = std::max(selector_ - 1, 0); }, ButtonOption::Ascii()) |
            vcenter,
    });
    bottom_ =
        Container::Horizontal({ Checkbox("Errors only", &showSuccess_) | vcenter, Renderer([] { return filler(); }),
                                bottomMiddle, Renderer([] { return filler(); }), clearButton | align_right });

    Add(Container::Vertical({ reportPanel_, bottom_ }));
}

ReportsPage::~ReportsPage()
{
}

bool ReportsPage::OnEvent(Event event)
{
    if (reportPanel_->ChildCount() == 0)
        return false;

    return ComponentBase::OnEvent(event);
}

Element ReportsPage::Render()
{
    while (reportQueue.size() > 0)
    {
        report_ = std::move(reportQueue.front());

        if (report_.size() != 0)
        {
            reportPanel_->Add(std::make_shared<JobReportPanel>(std::move(report_), &showSuccess_));
            if (reportPanel_->ChildCount() > 1)
                selector_ += 1;
        }

        reportQueue.pop();
    }

    if (reportPanel_->ChildCount() > 0)
        return vbox({
            reportPanel_->Render() | flex,
            separator(),
            bottom_->Render(),
        });

    return text("No report available, please run some tests first.") | center;
}

}  // namespace mil_preflight
