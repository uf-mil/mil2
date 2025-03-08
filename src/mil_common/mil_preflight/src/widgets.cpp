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
static std::queue<JobPanel::Summery> summeryQueue;

ActionBox::ActionBox(std::string&& name, std::vector<std::string>&& parameters):
    name_(std::move(name)),
    parameters_(std::move(parameters))
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

    if (focused || hovered_)
    {
        labelEle |= inverted;
    }

    if (focused)
    {
        labelEle |= bold;
    }

    auto element = hbox({
        text(checked_ ? "☑ " : "☐ "),
        labelEle,
        filler() | flex,
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
        if(Focused())
            checked_ = !checked_;
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

void ActionBox::onFinish(bool success, std::string&& summery)
{
    state_ = success ? State::SUCCESS : State::FAILED;
    summery_ = std::make_pair(success, std::move(summery));
    screen.PostEvent(Event::Character("ActionFinish"));
}

void ActionBox::onQuestion(std::shared_ptr<Question> question)
{
    questionQueue.push(question);
    screen.PostEvent(Event::Character("Question"));
}

TestPage::TestPage(std::string&& name, std::string&& plugin):
    name_(std::move(name)),
    plugin_(std::move(plugin))
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
    std::shared_ptr<ActionBox> action;

    if(currentAction_ > 0)
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(currentAction_-1));
        summery_.emplace(action->getName(),std::move(action->getSummery()));
    }
    else
    {
        summery_.clear();
    }

    while(currentAction_ < child->ChildCount())
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(currentAction_++));
        child->SetActiveChild(action);
        if(action->isChecked())
            return action;
        
        action->reset();
    }

    return nullptr;
}

std::shared_ptr<Action> TestPage::createAction(std::string&& name, std::vector<std::string>&& parameters)
{
    std::shared_ptr<ActionBox> action = std::make_shared<ActionBox>(std::move(name), std::move(parameters));
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

    if (focused || hovered_)
    {
        labelEle |= inverted;
    }

    if (focused)
    {
        labelEle |= bold;
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
        if(Focused())
            nextState();
        else
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
    if(currentTest_ > 0)
    {
        std::shared_ptr<TestPage> test = std::dynamic_pointer_cast<TestPage>(pagesContainer_->ChildAt(currentTest_-1));
        summery_.emplace(test->getName(), std::move(test->getSummery()));
    }
    else
    {
        summery_.clear();
    }

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

std::shared_ptr<Test> JobPanel::createTest(std::string&& name, std::string&& plugin)
{
    std::shared_ptr<TestPage> testPage = std::make_shared<TestPage>(std::move(name), std::move(plugin));
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

    Component bottom = Container::Horizontal({Checkbox("Select all", &selectAll_) | vcenter | flex, runButton});
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
        summeryQueue.push(panel_->getSummery());
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
            }, ButtonOption::Border()));
        }

        Component buttonsContainer = Container::Horizontal(buttons);
        Component closeButton = Button("X", [=]{
            selector_ = 0;
            dialog_->Detach();
            question->answer(-1);
        }, ButtonOption::Ascii());

        Component dialogContainer = Container::Vertical({closeButton, buttonsContainer});

        dialog_ = Renderer(dialogContainer ,[=]{
            return vbox({
                    closeButton->Render() | align_right,
                    separator(),
                    paragraph(question->getQuestion()) | flex, 
                    buttonsContainer->Render() | center
                }) | border;
        });

        ChildAt(0)->Add(dialog_);
        selector_ = 1;
        
        return true;
    }

    if(running_ && selector_ == 0)
    {
        return true;
    }

    return ComponentBase::OnEvent(event);
}

ReportPage::ReportPage()
{
    summeryPanel_ = Container::Vertical({});
    Component clearButton = Button("Clear", [=]{summeryPanel_->DetachAllChildren();}, ButtonOption::Border());
    Component bottom = Container::Horizontal({Checkbox("Show all", &showSuccess_) | vcenter | flex, clearButton});
    Add(Container::Vertical({
        summeryPanel_ | frame | flex,
        Renderer([]{return separator();}),
        bottom
    }));
}

ReportPage::~ReportPage()
{

}

Element ReportPage::Render()
{
    if(summeryQueue.size() > 0)
    {
        summeryPanel_->DetachAllChildren();
        summery_ = std::move(summeryQueue.front());

        Component testContainer = Container::Vertical({});
        for(const auto& testPair : summery_)
        {
            Component actionContainer = Container::Vertical({});
            bool success = true;
            for(const auto& actionPair: testPair.second)
            {
                Component summeryPanel = Renderer([&]{
                    return hbox({
                        text("  "),
                        paragraph(actionPair.second.second) //| color(actionPair.second.first ? Color::Green : Color::Red)
                    });
                });

                Component actionColp = Collapsible(actionPair.first, {summeryPanel});
                Component actionRender = Renderer(actionColp, [=]{
                    return actionColp->Render() | color(actionPair.second.first ? Color::Green : Color::Red);
                });
                if(actionPair.second.first)
                    actionContainer->Add(Maybe(actionRender, &showSuccess_));
                else
                    actionContainer->Add(actionRender);
            }
            summeryPanel_->Add(Collapsible(testPair.first, Renderer(actionContainer, [=]{
                return hbox({
                    text("  "),
                    actionContainer->Render()
                });
            })));
        }

        summeryQueue.pop();
    }

    if(summeryPanel_->ChildCount() > 0)
        return ChildAt(0)->Render();

    return text("No report avaiable, please run some tests first.") | center;
}

}