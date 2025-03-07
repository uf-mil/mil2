#include <sstream>
#include <queue>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

#include "mil_preflight/widgets.h"

using namespace ftxui;
extern ScreenInteractive screen;

namespace mil_preflight
{

static std::queue<std::shared_ptr<Question>> questionQueue;

ActionBox::ActionBox(std::string const& name, std::string const& parameters):
    name_(name),
    parameters_(parameters)
{
    
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

void ActionBox::onFinish(bool success, std::string const& info)
{
    state_ = success ? State::SUCCESS : State::FAILED;
    screen.PostEvent(Event::Character("ActionFinish"));
}

void ActionBox::onQuestion(std::shared_ptr<Question> question)
{
    questionQueue.push(question);
    screen.PostEvent(Event::Character("Question"));
}

TestPage::TestPage(std::string const& name, std::string const& plugin):
    name_(name),
    plugin_(plugin)
{
    Components comps;
    Add(Container::Vertical(comps, &selector_));
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

std::shared_ptr<Action> TestPage::createAction(std::string const& name, std::string const& parameters)
{
    std::shared_ptr<ActionBox> action = std::make_shared<ActionBox>(name, parameters);
    ChildAt(0)->Add(action);
    return action;
}

void TestPage::onFinish()
{
    currentAction_ = 0;
    screen.PostEvent(Event::Character("TestFinish"));
}

Element TestTab::Render() 
{
    bool focused = Focused();

    auto labelEle = text(test_->getName()); 

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

JobPanel::JobPanel()
{
    Components pages;
    Components tabs;

    tabsContainer_ = Container::Vertical(tabs, &selector_);
    pagesContainer_ = Container::Tab(pages,  &selector_);
    Component main = Container::Horizontal({tabsContainer_ | frame | vscroll_indicator, 
                                            Renderer([]{return separator(); }), 
                                            pagesContainer_ | frame | vscroll_indicator | flex});


    Add(main);
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

std::shared_ptr<Test> JobPanel::createTest(std::string const& name, std::string const& plugin)
{
    std::shared_ptr<TestPage> testPage = std::make_shared<TestPage>(name, plugin);
    pagesContainer_->Add(testPage);
    tabsContainer_->Add(std::make_shared<TestTab>(testPage));
    return testPage;
}

void JobPanel::onFinish()
{
    currentTest_ = 0;
    screen.PostEvent(Event::Character("JobFinish"));
}

JobPage::JobPage(std::string const& filePath):
    panel_(std::make_shared<JobPanel>())
{
    runner_.initialize(panel_, filePath);

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

    Component runButton = Button(buttonLabels_[0], [this]{
        bool expected = false;
        if(running_.compare_exchange_strong(expected, true))
        {
            panel_->TakeFocus();
            runner_.run(panel_);
        }
    }, buttonOption);

    Component bottom = Container::Horizontal({Checkbox("Select all", &selectAll_) | vcenter | flex, runButton | align_right});
    main_ = Container::Vertical({panel_ | flex, Renderer([]{return separator(); }), bottom});
    Add(Container::Tab({main_}, &selector_));
}

JobPage::~JobPage()
{

}

bool JobPage::OnEvent(Event event)
{
    if(event == Event::Character("JobFinish"))
    {
        running_ = false;
        return false;
    }

    if(event == Event::Character("TestFinish"))
        return false;
    
    if(event == Event::Character("ActionFinish"))
    {
        ticker_ ++;
        return false;
    }

    if(event == Event::Character("Question"))
    {

        Components buttons;
        std::shared_ptr<Question> question = questionQueue.front();
        questionQueue.pop();

        for(size_t i = 0; i < question->getOptionCount(); i++)
        {
            buttons.push_back(Button(question->getOpiton(i), [=]{
                selector_ = 0;
                dialog_->Detach();
                question->answer(i);
            }));
        }

        Component buttonsContainer = Container::Horizontal(buttons);
        dialog_ = Renderer(buttonsContainer ,[=]{
            return vbox({
                    text(question->getQuestion()) | flex, 
                    buttonsContainer->Render() | align_right
                });
        });

        ChildAt(0)->Add(dialog_);
        selector_ = 1;
        
        return true;
    }

    return ComponentBase::OnEvent(event);
}

}