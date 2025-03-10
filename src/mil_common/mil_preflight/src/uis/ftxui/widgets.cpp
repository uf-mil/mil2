#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <functional>
#include <queue>
#include <sstream>
#include <stack>

#include "mil_preflight/uis/ftxui/dialog.h"
#include "mil_preflight/uis/ftxui/reportsPage.h"
#include "mil_preflight/uis/ftxui/testsPage.h"

using namespace ftxui;
extern ScreenInteractive screen;

namespace mil_preflight
{

static std::queue<Job::Report> reportQueue;

static std::queue<std::shared_ptr<Dialog>> questionQueue;

class ActionBox : public ComponentBase, public Action
{
public:
  ActionBox(std::string&& name, std::vector<std::string>&& parameters);
  ~ActionBox();

  bool isChecked() const
  {
    return checked_;
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
    return name_;
  }
  std::vector<std::string> const& getParameters() const final
  {
    return parameters_;
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
  std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options) final;
};

ActionBox::ActionBox(std::string&& name, std::vector<std::string>&& parameters)
  : name_(std::move(name)), parameters_(std::move(parameters))
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

  auto labelEle = text(name_);

  if (focused || hovered_)
  {
    labelEle |= inverted;
  }

  if (focused)
  {
    labelEle |= bold;
  }

  auto element = hbox(
      { text(checked_ ? "☑ " : "☐ "), labelEle, filler() | flex, text(indicator) | color(textColor) | align_right });

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

  if (event.mouse().button == Mouse::Left && event.mouse().motion == Mouse::Pressed)
  {
    if (Focused())
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

std::shared_future<int> ActionBox::onQuestion(std::string&& question, std::vector<std::string>&& options)
{
  std::shared_ptr<std::promise<int>> feedback = std::make_shared<std::promise<int>>();

  Dialog::Option option;
  option.title = "Question for action" + name_;
  option.question = std::move(question);
  option.buttonLabels = std::move(options);
  option.onClose = [=](int index) mutable { feedback->set_value(index); };

  std::shared_ptr<Dialog> dialog = std::make_shared<Dialog>(std::move(option));
  questionQueue.push(dialog);
  screen.PostEvent(Event::Character("Question"));
  return feedback->get_future().share();
}

class ActionList : public ComponentBase, public Test
{
public:
  using History = std::pair<size_t, bool>;

  ActionList(std::string&& name, std::string&& plugin);
  ~ActionList();

  size_t actionCount()
  {
    return ChildAt(0)->ChildCount();
  }

  size_t isChecked();
  inline void saveAndCheck(std::stack<History>& historys);
  inline void saveAndUncheck(std::stack<History>& historys);
  inline void restore(std::stack<History>& historys);
  inline void check();
  inline void uncheck();

  std::string const& getName() const final
  {
    return name_;
  };
  std::string const& getPlugin() const final
  {
    return plugin_;
  };

private:
  int selector_ = 0;
  bool checked_ = 0;
  std::string name_;
  std::string plugin_;
  std::atomic<size_t> currentAction_ = 0;
  Box box_;

  std::optional<std::reference_wrapper<Action>> nextAction() final;
  std::optional<std::reference_wrapper<Action>> createAction(std::string&& name,
                                                             std::vector<std::string>&& parameters) final;
  void onFinish(Test::Report const& report) final;
};

ActionList::ActionList(std::string&& name, std::string&& plugin) : name_(std::move(name)), plugin_(std::move(plugin))
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
  for (size_t i = 0; i < child->ChildCount(); i++)
  {
    std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
    if (action->isChecked())
      count++;
  }
  return count;
}

void ActionList::saveAndCheck(std::stack<History>& historys)
{
  Component child = ChildAt(0);
  for (size_t i = 0; i < child->ChildCount(); i++)
  {
    std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
    if (!action->isChecked())
    {
      historys.emplace(i, false);
      action->check();
    }
  }
}

void ActionList::check()
{
  Component child = ChildAt(0);
  for (size_t i = 0; i < child->ChildCount(); i++)
  {
    std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
    action->check();
  }
}

void ActionList::restore(std::stack<History>& historys)
{
  Component child = ChildAt(0);
  while (historys.size() > 0)
  {
    History history = std::move(historys.top());
    std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(history.first));
    if (history.second)
      action->check();
    else
      action->uncheck();
    historys.pop();
  }
}

void ActionList::saveAndUncheck(std::stack<History>& historys)
{
  Component child = ChildAt(0);
  for (size_t i = 0; i < child->ChildCount(); i++)
  {
    std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
    if (action->isChecked())
    {
      historys.emplace(i, true);
      action->uncheck();
    }
  }
}

void ActionList::uncheck()
{
  Component child = ChildAt(0);
  for (size_t i = 0; i < child->ChildCount(); i++)
  {
    std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
    action->uncheck();
  }
}

std::optional<std::reference_wrapper<Action>> ActionList::nextAction()
{
  Component child = ChildAt(0);

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

std::optional<std::reference_wrapper<Action>> ActionList::createAction(std::string&& name,
                                                                       std::vector<std::string>&& parameters)
{
  std::shared_ptr<ActionBox> action = std::make_shared<ActionBox>(std::move(name), std::move(parameters));
  ChildAt(0)->Add(action);
  return *action;
}

void ActionList::onFinish([[maybe_unused]] Test::Report const& report)
{
  currentAction_ = 0;
  screen.PostEvent(Event::Character("TestFinish"));
}

class TestTab : public ComponentBase
{
public:
  enum class State
  {
    Checked,
    Indeterminate,
    Unchecked
  };

  TestTab(std::shared_ptr<ActionList> test) : test_(test)
  {
  }
  ~TestTab()
  {
  }

  State getState() const
  {
    return state_;
  }
  void check();
  void uncheck();

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

  bool Focusable() const final
  {
    return true;
  }
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

  char const* prefix;
  size_t nChecked = test_->isChecked();
  size_t nActions = test_->actionCount();
  if (nChecked == 0)
  {
    state_ = State::Unchecked;
    prefix = "☐ ";
  }
  else if (nChecked == nActions)
  {
    state_ = State::Checked;
    prefix = "☑ ";
  }
  else
  {
    state_ = State::Indeterminate;
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

  if (event.mouse().button == Mouse::Left && event.mouse().motion == Mouse::Pressed)
  {
    if (Focused())
      nextState();
    else
      TakeFocus();
    return true;
  }

  return false;
}

void TestTab::check()
{
  if (state_ != State::Checked)
  {
    test_->saveAndCheck(historys_);
  }
}

void TestTab::uncheck()
{
  if (state_ == State::Checked)
  {
    if (historys_.size() > 0)
      test_->restore(historys_);
  }
}

void TestTab::nextState()
{
  if (state_ == State::Unchecked)
  {
    if (historys_.size() > 0)
      test_->restore(historys_);
    else
      test_->check();
  }
  else if (state_ == State::Indeterminate)
  {
    while (historys_.size() > 0)
      historys_.pop();

    if (toggle_)
      test_->saveAndCheck(historys_);
    else
      test_->saveAndUncheck(historys_);
    toggle_ = !toggle_;
  }
  else if (state_ == State::Checked)
  {
    if (historys_.size() > 0)
      test_->restore(historys_);
    else
      test_->uncheck();
  }
}

TestsPage::TestsPage(std::string const& filePath)
{
  Components pages;
  Components tabs;

  tabsContainer_ = Container::Vertical(tabs, &selector_);
  pagesContainer_ = Container::Tab(pages, &selector_);
  main_ = Container::Horizontal({ tabsContainer_ | frame | vscroll_indicator, Renderer([] { return separator(); }),
                                  pagesContainer_ | frame | vscroll_indicator | flex });

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
        run();
      },
      buttonOption);

  CheckboxOption option = CheckboxOption::Simple();
  option.on_change = [=]
  {
    for (size_t i = 0; i < pagesContainer_->ChildCount(); i++)
    {
      std::shared_ptr<TestTab> tab = std::dynamic_pointer_cast<TestTab>(tabsContainer_->ChildAt(i));
      if (selectAll_)
        tab->check();
      else
        tab->uncheck();
    }
    screen.PostEvent(Event::Custom);
  };
  Component checkBox = Checkbox("Select all", &selectAll_, option);
  Component bottom = Container::Horizontal({ checkBox | vcenter | flex, runButton });
  Add(Container::Vertical({ main_ | flex, Renderer([] { return separator(); }), bottom }));

  initialize(filePath);
}

TestsPage::~TestsPage()
{
  if (running_)
    cancel();
}

Element TestsPage::Render()
{
  size_t nChecked = 0;
  for (size_t i = 0; i < tabsContainer_->ChildCount(); i++)
  {
    std::shared_ptr<TestTab> tab = std::dynamic_pointer_cast<TestTab>(tabsContainer_->ChildAt(i));
    if (tab->getState() == TestTab::State::Checked)
      nChecked++;
  }

  selectAll_ = (nChecked == tabsContainer_->ChildCount());

  return ComponentBase::Render();
}

std::optional<std::reference_wrapper<Test>> TestsPage::nextTest()
{
  running_ = true;

  while (currentTest_ < tabsContainer_->ChildCount())
  {
    std::shared_ptr<TestTab> test = std::dynamic_pointer_cast<TestTab>(tabsContainer_->ChildAt(currentTest_++));
    if (test->getState() != TestTab::State::Unchecked)
    {
      tabsContainer_->SetActiveChild(tabsContainer_->ChildAt(currentTest_ - 1));
      return *std::dynamic_pointer_cast<ActionList>(pagesContainer_->ChildAt(currentTest_ - 1));
    }
  }
  return std::nullopt;
}

std::optional<std::reference_wrapper<Test>> TestsPage::createTest(std::string&& name, std::string&& plugin)
{
  std::shared_ptr<ActionList> testPage = std::make_shared<ActionList>(std::move(name), std::move(plugin));
  pagesContainer_->Add(testPage);
  tabsContainer_->Add(std::make_shared<TestTab>(testPage));
  return *testPage;
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

  if (event == Event::Character("Question"))
  {
    std::shared_ptr<Dialog> dialog = questionQueue.front();
    questionQueue.pop();
    dialog->show(this);

    return true;
  }

  if (running_ && main_->Focused())
    return false;

  return ComponentBase::OnEvent(event);
}

ReportsPage::ReportsPage()
{
  reportPage_ = Container::Tab({}, &selector_);
  Component clearButton = Button("Delete", [=] { 
    if(reportPage_->ChildCount() > 0)
      reportPage_->ChildAt(selector_)->Detach();
      selector_ -= 1; 
  }, ButtonOption::Border());
  Component bottomMiddle = Container::Horizontal({
      Button(
          "<", [=] { selector_ = std::min(selector_ + 1, static_cast<int>(reportPage_->ChildCount() - 1)); },
          ButtonOption::Ascii()) |
          vcenter,
      Renderer(
          [=]
          {
            return text(std::to_string(reportPage_->ChildCount() - selector_) + "/" +
                        std::to_string(reportPage_->ChildCount()));
          }) |
          vcenter,
      Button(
          ">", [=] { selector_ = std::max(selector_ - 1, 0); }, ButtonOption::Ascii()) |
          vcenter,
  });
  Component bottom =
      Container::Horizontal({ Checkbox("Show all", &showSuccess_) | vcenter, Renderer([] { return filler(); }),
                              bottomMiddle, Renderer([] { return filler(); }), clearButton | align_right });

  Add(Container::Vertical({ reportPage_ | flex, Renderer([] { return separator(); }), bottom }));
}

ReportsPage::~ReportsPage()
{
}

bool ReportsPage::OnEvent(Event event)
{
  if(reportPage_->ChildCount() == 0)
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
      Component reportPanel_ = Container::Vertical({});
      for (auto const& testPair : report_)
      {
        Component actionContainer = Container::Vertical({});
        for (auto const& actionPair : testPair.second)
        {
          Component summeryPanel = Renderer(
              [&]
              {
                return hbox({
                    text("  "),
                    paragraph(actionPair.second.summery)  //| color(actionPair.second.first ? Color::Green : Color::Red)
                });
              });

          Component actionColp = Collapsible(actionPair.first, { summeryPanel });
          Component actionRender =
              Renderer(actionColp, [=]
                       { return actionColp->Render() | color(actionPair.second.success ? Color::Green : Color::Red); });
          if (actionPair.second.success)
            actionContainer->Add(Maybe(actionRender, &showSuccess_));
          else
            actionContainer->Add(actionRender);
        }

        reportPanel_->Add(
            Collapsible(testPair.first,
                        Renderer(actionContainer, [=] { return hbox({ text("  "), actionContainer->Render() }); })));
      }

      reportPage_->Add(reportPanel_ | flex | frame);
    }

    reportQueue.pop();
  }

  if (reportPage_->ChildCount() > 0)
    return ChildAt(0)->Render();

  return text("No report available, please run some tests first.") | center;
}

}  // namespace mil_preflight
