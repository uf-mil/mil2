
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

#include "mil_preflight/widgets.h"

using namespace ftxui;
extern ScreenInteractive screen;

namespace mil_preflight
{

ActionBox::ActionBox(boost::json::key_value_pair const& pair)
{
    name_ = pair.key();
    parameters_ = pair.value().as_string();
}

ActionBox::~ActionBox()
{

}

Element ActionBox::Render() 
{
    bool focused = Focused();
    // bool active = Active();

    const char* indicator;
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
        indicator ="   ";
        textColor = Color::White;
        break;
    }

    auto labelEle = text(name_); 

    if (focused)
    {
        labelEle |= inverted;
    }

    auto element = hbox({
        text(checked_ ? "☑ " : "☐ "),
        labelEle | flex,
        text(indicator) | color(textColor) | align_right
    });

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

    if (event.mouse().button == Mouse::Left &&
        event.mouse().motion == Mouse::Pressed) 
    {
        checked_ = !checked_;
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

void ActionBox::onSuccess(std::string const& info)
{
    state_ = State::SUCCESS;
    screen.PostEvent(Event::Custom);
}
void ActionBox::onFail(std::string const& info)
{
    state_ = State::FAILED;
    screen.PostEvent(Event::Custom);
}

TestPage::TestPage(boost::json::key_value_pair const& pair)
{
    name_ = pair.key();
    boost::json::object obj = pair.value().as_object();
    plugin_ = obj.at("plugin").as_string();

    Components tests;
    for(const auto& actionPair: obj.at("actions").as_object())
    {
        tests.push_back(std::make_shared<ActionBox>(actionPair));
    }

    Add(Container::Vertical(tests, &selector_));
}

TestPage::~TestPage()
{

}

size_t TestPage::isChecked()
{
    size_t count = 0;
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
        if(action->isChecked())
            count++;
    }
    return count;
}

void TestPage::saveAndCheck(std::stack<History>& historys)
{
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
        if(!action->isChecked())
        {
            historys.emplace(i,false);
            action->check();
        }
    }
}

void TestPage::check()
{
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
        action->check();
    }
}

void TestPage::restore(std::stack<History>& historys)
{
    Component child = ChildAt(0);
    while(historys.size() > 0)
    {
        History history = std::move(historys.top());
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(history.first));
        if(history.second)
            action->check();
        else
            action->uncheck();
        historys.pop();
    }
}

void TestPage::saveAndUncheck(std::stack<History>& historys)
{
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
        if(action->isChecked())
        {
            historys.emplace(i, true);
            action->uncheck();
        }
    }
}

void TestPage::uncheck()
{
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
        action->uncheck();
    }
}

std::shared_ptr<Action> TestPage::nextAction()
{
    Component child = ChildAt(0);
    while(currentAction_ < child->ChildCount())
    {

        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(currentAction_++));
        child->SetActiveChild(action);
        if(action->isChecked())
            return action;
    }

    return nullptr;
}

void TestPage::onFinish()
{
    currentAction_ = 0;
}

Element TestTab::Render() 
{
    bool focused = Focused();

    auto labelEle = text(test_->name_); 

    if (focused)
    {
        labelEle |= inverted;
    }

    const char* prefix;
    size_t nChecked = test_->isChecked();
    size_t nActions = test_->actionCount();
    if(nChecked == 0)
    {
        state_ = State::Unchecked;
        prefix = "☐ ";
    }
    else if(nChecked == nActions)
    {
        state_ = State::Checked;
        prefix = "☑ ";
    }
    else
    {
        state_ = State::Indeterminate;
        prefix = "▣ ";
    }

    auto element = hbox({
        text(prefix),
        labelEle
    });

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
        nextState();
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

    if (event.mouse().button == Mouse::Left &&
        event.mouse().motion == Mouse::Pressed) 
    {
        nextState();
        TakeFocus();
        return true;
    }

    return false;
}

void TestTab::nextState()
{
    if(state_ == State::Unchecked)
    {
        if(historys_.size() > 0)
            test_->restore(historys_);
        else
            test_->check();
    }
    else if(state_ == State::Indeterminate)
    {
        while(historys_.size() > 0)
            historys_.pop();

        if(toggle_)
            test_->saveAndCheck(historys_);
        else
            test_->saveAndUncheck(historys_);
        toggle_ = !toggle_;
    }
    else if(state_ == State::Checked)
    {
        if(historys_.size() > 0)
            test_->restore(historys_);
        else
            test_->uncheck();
    }
}

JobPanel::JobPanel(boost::json::value const& value)
{
    Components pages;
    Components tabs;
    for(const auto& testPair: value.as_object())
    {
        std::shared_ptr<TestPage> testPage = std::make_shared<TestPage>(testPair);
        std::shared_ptr<TestTab> testTab = std::make_shared<TestTab>(testPage);
        pages.push_back(testPage);
        tabs.push_back(testTab);
    }

    tabsContainer_ = Container::Vertical(tabs, &selector_);
    pagesContainer_ = Container::Tab(pages,  &selector_);
    Component main = Container::Horizontal({tabsContainer_ | frame | vscroll_indicator, 
                                            Renderer([]{return separator(); }), 
                                            pagesContainer_ | frame | vscroll_indicator | flex});

    main |= CatchEvent([&](Event event){
        if(event == Event::Custom)
        ticker_ ++;
        
        if(running_)
        {
            return true;
        }
    
        return false;
    });

    ButtonOption buttonOption = ButtonOption::Simple();
    buttonOption.transform = [&](const EntryState& s) 
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
    Component runButton = Button(buttonLabels_[0], [&]{
        bool expected = false;
        if(running_.compare_exchange_strong(expected, true))
        {
            ticker_ = 0;
            currentTest_ = 0;
            tabsContainer_->TakeFocus();
            runner_.run(shared_from_this());
        }
    }, buttonOption);

    CheckboxOption checkboxOption = CheckboxOption::Simple();
    checkboxOption.on_change = [&]{
        if(selectAll_)
        {
            for(size_t i=0;i<pagesContainer_->ChildCount();i++)
            {
                std::dynamic_pointer_cast<TestPage>(pagesContainer_->ChildAt(i))->check();
            }
        }
    };
    Component bottom = Container::Horizontal({Checkbox("Select all", &selectAll_, checkboxOption) | vcenter | flex, runButton | align_right});

    Add(Container::Vertical({main | flex, Renderer([]{return separator(); }), bottom}));
}

JobPanel::~JobPanel()
{

}

std::shared_ptr<Test> JobPanel::nextTest()
{
    while(currentTest_ < tabsContainer_->ChildCount())
    {
        std::shared_ptr<TestTab> test = std::dynamic_pointer_cast<TestTab>(tabsContainer_->ChildAt(currentTest_++));
        if(test->getState() != TestTab::State::Unchecked)
        {
            tabsContainer_->SetActiveChild(tabsContainer_->ChildAt(currentTest_-1));
            return std::dynamic_pointer_cast<TestPage>(pagesContainer_->ChildAt(currentTest_-1));
        }
    }
    return nullptr;
}

void JobPanel::onFinish()
{
    running_ = false;
}

}