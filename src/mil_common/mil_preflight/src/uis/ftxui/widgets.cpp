#include <sstream>
#include <queue>
#include <functional>
#include <stack>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>

#include "mil_preflight/uis/ftxui/testPage.h"
#include "mil_preflight/uis/ftxui/reportPage.h"
#include "mil_preflight/uis/ftxui/dialog.h"

using namespace ftxui;
extern ScreenInteractive screen;

namespace mil_preflight
{

static std::queue<Job::Report> reportQueue;

class QuestionDialog: public Dialog, public Question
{
    public:
    QuestionDialog(std::string const& title, std::string&& question, std::vector<std::string>&& options):
        Dialog(title, std::move(question), std::move(options))
    {

    }

    ~QuestionDialog()
    {

    }

    private:

    void onClose(int index) final
    {
        answer(index);
    }
};

static std::queue<std::shared_ptr<QuestionDialog>> questionQueue;

class ActionBox: public ComponentBase, public Action
{
    public:

    ActionBox(std::string&& name, std::vector<std::string>&& parameters);
    ~ActionBox();

    bool isChecked() const { return checked_; }
    void check() { checked_ = true; }
    void uncheck() { checked_ = false; }
    void reset() {state_ = State::NONE;}

    std::string const& getName() const final { return name_; }
    std::vector<std::string> const& getParameters() const final {return parameters_; }

    private:

    enum class State
    {
        NONE,
        RUNNING,
        SUCCESS,
        FAILED
    };

    bool checked_ = false;
    bool forceCheck_ = false;
    bool hovered_ = false;
    Box box_;
    std::atomic<State> state_ = State::NONE;
    std::string name_;
    std::vector<std::string> parameters_;

    Element Render() final;
    bool OnEvent(Event event) final;
    inline bool OnMouseEvent(Event event);
    bool Focusable() const final;

    void onStart() final;
    void onFinish(Action::Report const& report) final;
    std::shared_ptr<Question> onQuestion(std::string&& question, std::vector<std::string>&& options) final;
};

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

void ActionBox::onFinish(Action::Report const& report)
{
    state_ = report.success ? State::SUCCESS : State::FAILED;
    screen.PostEvent(Event::Character("ActionFinish"));
}

std::shared_ptr<Question> ActionBox::onQuestion(std::string&& question, std::vector<std::string>&& options)
{
    std::shared_ptr<QuestionDialog> dialog = std::make_shared<QuestionDialog>("Question for action: " + name_ ,std::move(question), std::move(options));
    questionQueue.push(dialog);
    screen.PostEvent(Event::Character("Question"));
    return dialog;
}

class ActionList: public ComponentBase, public Test
{
  public:
  using History = std::pair<size_t, bool>;

  ActionList(std::string&& name, std::string&& plugin);
  ~ActionList();

  size_t actionCount() {return ChildAt(0)->ChildCount();}

  size_t isChecked();
  inline void saveAndCheck(std::stack<History>& historys);
  inline void saveAndUncheck(std::stack<History>& historys);
  inline void restore(std::stack<History>& historys);
  inline void check();
  inline void uncheck();

  std::string const& getName() const final {return name_;};
  std::string const& getPlugin() const final {return plugin_;};

  private:
  int selector_ = 0;
  bool checked_ = 0;
  std::string name_;
  std::string plugin_;
  std::atomic<size_t> currentAction_ = 0;
  Box box_;

  std::shared_ptr<Action> nextAction() final;
  std::shared_ptr<Action> createAction(std::string&& name, std::vector<std::string>&& parameters) final;
  void onFinish(Test::Report const& report) final;
};

ActionList::ActionList(std::string&& name, std::string&& plugin):
    name_(std::move(name)),
    plugin_(std::move(plugin))
{
    Components comps;
    Add(Container::Vertical(comps, &selector_));
}

ActionList::~ActionList()
{

}

size_t ActionList::isChecked()
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

void ActionList::saveAndCheck(std::stack<History>& historys)
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

void ActionList::check()
{
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
        action->check();
    }
}

void ActionList::restore(std::stack<History>& historys)
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

void ActionList::saveAndUncheck(std::stack<History>& historys)
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

void ActionList::uncheck()
{
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
        action->uncheck();
    }
}

std::shared_ptr<Action> ActionList::nextAction()
{
    Component child = ChildAt(0);

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

std::shared_ptr<Action> ActionList::createAction(std::string&& name, std::vector<std::string>&& parameters)
{
    std::shared_ptr<ActionBox> action = std::make_shared<ActionBox>(std::move(name), std::move(parameters));
    ChildAt(0)->Add(action);
    return action;
}

void ActionList::onFinish([[maybe_unused]] Test::Report const& report)
{
    currentAction_ = 0;
    screen.PostEvent(Event::Character("TestFinish"));
}

class TestTab: public ComponentBase
{
  public:
  enum class State
  {
    Checked,
    Indeterminate,
    Unchecked
  };

  TestTab(std::shared_ptr<ActionList> test):test_(test){}
  ~TestTab(){}

  State getState() const {return state_;}

  private:

  bool hovered_ = false;
  State state_ = State::Unchecked;
  bool toggle_ = false;
  std::shared_ptr<ActionList> test_;
  std::stack<ActionList::History> historys_;
  Box box_;

  Element Render() final;
  bool OnEvent(Event event) final;
  bool OnMouseEvent(Event event);

  inline void nextState();

  bool Focusable() const final { return true; } 

};

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

class TestPanel: public ComponentBase, public Job //, public std::enable_shared_from_this<Job>
{
  public:

  TestPanel();
  ~TestPanel();

  private:

  Component tabsContainer_;
  Component pagesContainer_;

  int selector_ = 0;
  std::atomic<size_t> currentTest_ = 0;
  std::atomic<bool> running_ = false;

  std::shared_ptr<Test> nextTest() final;
  std::shared_ptr<Test> createTest(std::string&& name, std::string&& plugin) final;
  void onFinish(Job::Report&& report) final;

  bool OnEvent(Event event) final
  {
    if(running_)
        return true;
    
    return ComponentBase::OnEvent(event);
  }

};

TestPanel::TestPanel()
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

TestPanel::~TestPanel()
{

}

std::shared_ptr<Test> TestPanel::nextTest()
{
    running_ = true;

    while(currentTest_ < tabsContainer_->ChildCount())
    {
        std::shared_ptr<TestTab> test = std::dynamic_pointer_cast<TestTab>(tabsContainer_->ChildAt(currentTest_++));
        if(test->getState() != TestTab::State::Unchecked)
        {
            tabsContainer_->SetActiveChild(tabsContainer_->ChildAt(currentTest_-1));
            return std::dynamic_pointer_cast<ActionList>(pagesContainer_->ChildAt(currentTest_-1));
        }
    }
    return nullptr;
}

std::shared_ptr<Test> TestPanel::createTest(std::string&& name, std::string&& plugin)
{
    std::shared_ptr<ActionList> testPage = std::make_shared<ActionList>(std::move(name), std::move(plugin));
    pagesContainer_->Add(testPage);
    tabsContainer_->Add(std::make_shared<TestTab>(testPage));
    return testPage;
}

void TestPanel::onFinish(Job::Report&& report)
{
    running_ = false;
    currentTest_ = 0;
    reportQueue.push(std::move(report));
    screen.PostEvent(Event::Character("JobFinish"));
}

TestPage::TestPage(std::string const& filePath):
    panel_(std::make_shared<TestPanel>())
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
    Add(main_);
}

TestPage::~TestPage()
{

}

Element TestPage::Render()
{
    return ChildAt(0)->Render();
}

bool TestPage::OnEvent(Event event)
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

        std::shared_ptr<QuestionDialog> dialog = questionQueue.front();
        questionQueue.pop();
        dialog->show(this);
        
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
    if(reportQueue.size() > 0)
    {
        summeryPanel_->DetachAllChildren();
        report_ = std::move(reportQueue.front());

        Component testContainer = Container::Vertical({});
        for(const auto& testPair : report_)
        {
            Component actionContainer = Container::Vertical({});
            // bool success = true;
            for(const auto& actionPair: testPair.second)
            {
                Component summeryPanel = Renderer([&]{
                    return hbox({
                        text("  "),
                        paragraph(actionPair.second.summery) //| color(actionPair.second.first ? Color::Green : Color::Red)
                    });
                });

                Component actionColp = Collapsible(actionPair.first, {summeryPanel});
                Component actionRender = Renderer(actionColp, [=]{
                    return actionColp->Render() | color(actionPair.second.success ? Color::Green : Color::Red);
                });
                if(actionPair.second.success)
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

        reportQueue.pop();
    }

    if(summeryPanel_->ChildCount() > 0)
        return ChildAt(0)->Render();

    return text("No report avaiable, please run some tests first.") | center;
}

}