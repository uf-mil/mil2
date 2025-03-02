// Copyright 2020 Arthur Sonzogni. All rights reserved.
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.
#include <memory>  // for shared_ptr, __shared_ptr_access
#include <string>  // for string, basic_string, operator+, to_string
#include <vector>  // for vector
#include <filesystem>
#include <fstream>
#include <stack>
 
#include "ftxui/component/captured_mouse.hpp"      // for ftxui
#include "ftxui/component/component.hpp"           // for Radiobox, Renderer
#include "ftxui/component/component_base.hpp"      // for ComponentBase
#include "ftxui/component/screen_interactive.hpp"  // for ScreenInteractive
#include "ftxui/dom/elements.hpp"  // for operator|, Element, size, border, frame, HEIGHT, LESS_THAN
 
#include "boost/json.hpp"

#include "mil_preflight/job.h"

using namespace ftxui;

ScreenInteractive screen = ScreenInteractive::Fullscreen();

namespace mil_preflight
{

class ActionBox: public ComponentBase, public Action
{
    public:
    friend class TestPage;
    ActionBox(boost::json::key_value_pair const& pair)
    {
      name_ = pair.key();
      parameters_ = pair.value().as_string();
    }

    ~ActionBox()
    {

    }

    private:
    Element Render() override 
    {
      bool focused = Focused();
      // bool active = Active();

      const char* indicator;
      switch (state_)
      {
      case State::RUNNING:
          indicator = " ▶ ";
          break;
      case State::SUCCESS:
          indicator = " ✔ ";
          break;
      case State::FAILED:
          indicator = " ✘ ";
          break;
      default:
          indicator ="   ";
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
          text(indicator) | align_right
      });

      return element | (focused ? focus : nothing) | reflect(box_);
    }

    bool OnEvent(Event event) override 
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
        // event.screen_->PostEvent(Event::Special("onChange"));
        TakeFocus();
        return true;
      }

      return false;
    }
  
    bool OnMouseEvent(Event event) 
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
        // event.screen_->PostEvent(Event::Character("onChange"));
        TakeFocus();
        return true;
      }
  
      return false;
    }
  
    bool Focusable() const final { return true; }  

    bool isChecked() const { return checked_; }
    void check() { checked_ = true; }
    void uncheck() { checked_ = false; }

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

    void onStart() final
    {
      state_ = State::RUNNING;
    }

    void onSuccess(std::string const& info) final
    {
      state_ = State::SUCCESS;
      screen.PostEvent(Event::Custom);
    }
    void onFail(std::string const& info) final
    {
      state_ = State::FAILED;
      screen.PostEvent(Event::Custom);
    }

};

class TestPage: public ComponentBase, public Test
{
  public:
  using History = std::pair<size_t, bool>;

  TestPage(boost::json::key_value_pair const& pair)
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

  ~TestPage()
  {

  }

  size_t actionCount()
  {
    return ChildAt(0)->ChildCount();
  }

  size_t isChecked()
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

  void saveAndCheck(std::stack<History>& historys)
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

  void check()
  {
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
      std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
      action->check();
    }
  }

  void restore(std::stack<History>& historys)
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

  void saveAndUncheck(std::stack<History>& historys)
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

  void uncheck()
  {
    Component child = ChildAt(0);
    for(size_t i=0;i<child->ChildCount();i++)
    {
      std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(i));
      action->uncheck();
    }
  }

  private:
  int selector_ = 0;
  bool checked_ = 0;
  size_t currentAction_ = 0;
  Box box_;

  std::shared_ptr<Action> nextAction() final
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
  void onFinish() final
  {
    currentAction_ = 0;
  }

};

class TestTab: public ComponentBase
{
  public:
  enum class State
  {
    Checked,
    Indeterminate,
    Unchecked
  };

  TestTab(std::shared_ptr<TestPage> test):test_(test)
  {

  }
  ~TestTab()
  {

  }

  State getState() const
  {
    return state_;
  }

  private:

  bool hovered_ = false;
  State state_ = State::Unchecked;
  bool toggle_ = false;
  std::shared_ptr<TestPage> test_;
  std::stack<TestPage::History> historys_;
  Box box_;

  Element Render() override 
  {
      bool focused = Focused();
      // bool active = Active();

      auto labelEle = text(test_->name_); 

      if (focused)
      {
          labelEle |= inverted;
      }

      const char* prefix;
      size_t nChecked = test_->isChecked();
      size_t nActions = test_->actionCount();
      // lastState_ = state_;
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

  bool OnEvent(Event event) override 
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

  bool OnMouseEvent(Event event)
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

  void nextState()
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

  bool Focusable() const final { return true; } 

};

class TestPanel: public ComponentBase, public Job, public std::enable_shared_from_this<Job>
{
  public:
  TestPanel(boost::json::value const& value)
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
    Component main = Container::Horizontal({tabsContainer_ , 
                                            Renderer([]{return separator(); }), 
                                            pagesContainer_ | flex});
    
    main |= CatchEvent([&](Event event){
      if(event == Event::Custom)
        ticker_ ++;
      
      if(currentTest_!=0)
      {
        return true;
      }
        
      return false;
    });
    
    ButtonOption option = ButtonOption::Simple();
    option.transform = [&](const EntryState& s) 
    {
      auto element = (running_ ? text(buttonLabels_[ticker_ % 3 + 1]) : text(s.label)) | border;
      if (s.active) {
        element |= bold;
      }
      if (s.focused) {
        element |= inverted;
      }
      return element;
    };
    Component bottom = Button(buttonLabels_[0], [&]{
      bool expected = false;
      if(running_.compare_exchange_strong(expected, true))
      {
        ticker_ = 0;
        currentTest_ = 0;
        tabsContainer_->TakeFocus();
        runner_.run(shared_from_this());
      }
    }, option)|align_right;
    
    Add(Container::Vertical({main | flex, Renderer([]{return separator(); }), bottom}));
  }

  ~TestPanel()
  {

  }

  private:

  Component tabsContainer_;
  Component pagesContainer_;
  int selector_ = 0;
  size_t currentTest_ = 0;
  std::atomic<bool> running_ = false;
  const std::string buttonLabels_[4] = {" Run ", " ·   ", "  ·  ", "   · "};
  JobRunner runner_;
  size_t ticker_ = 0;

  std::shared_ptr<Test> nextTest()
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

  void onFinish() final
  {
    running_ = false;
  }

};
}


int main() 
{
  auto binPath = std::filesystem::canonical("/proc/self/exe").parent_path();
  std::string fileName = binPath / ".." / "cfg" / "config.json";

  std::ifstream file(fileName);

  boost::json::value data = boost::json::parse(file);

  file.close();

  auto test = std::make_shared<mil_preflight::TestPanel>(data);
  screen.Loop(test);
 
  return 0;
}