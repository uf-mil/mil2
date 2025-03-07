#pragma once

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include <boost/json.hpp>

#include <atomic>
#include <stack>
#include <mutex>
#include <condition_variable>

#include "mil_preflight/job.h"

namespace mil_preflight
{
using namespace ftxui;
class ActionBox: public ComponentBase, public Action
{
    public:
    // friend class TestPage;
    ActionBox(std::string const& name, std::string const& parameters);
    ~ActionBox();

    bool isChecked() const { return checked_; }
    void check() { checked_ = true; }
    void uncheck() { checked_ = false; }
    std::string const& getName() const final { return name_; }
    std::string const& getParameter() const final {return parameters_; }

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
    std::string parameters_;

    Element Render() final;
    bool OnEvent(Event event) final;
    inline bool OnMouseEvent(Event event);
    bool Focusable() const final;

    void onStart() final;
    void onFinish(bool success, std::string const& summery) final;
    void onQuestion(std::shared_ptr<Question> question) final;
};

class TestPage: public ComponentBase, public Test
{
  public:
  using History = std::pair<size_t, bool>;

  TestPage(std::string const& name, std::string const& plugin);
  ~TestPage();

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
  std::shared_ptr<Action> createAction(std::string const& name, std::string const& parameters) final;
  void onFinish() final;

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

  TestTab(std::shared_ptr<TestPage> test):test_(test){}
  ~TestTab(){}

  State getState() const {return state_;}

  private:

  bool hovered_ = false;
  State state_ = State::Unchecked;
  bool toggle_ = false;
  std::shared_ptr<TestPage> test_;
  std::stack<TestPage::History> historys_;
  Box box_;

  Element Render() final;
  bool OnEvent(Event event) final;
  bool OnMouseEvent(Event event);

  inline void nextState();

  bool Focusable() const final { return true; } 

};


class JobPanel: public ComponentBase, public Job //, public std::enable_shared_from_this<Job>
{
  public:
  JobPanel();
  ~JobPanel();

  private:

  Component tabsContainer_;
  Component pagesContainer_;

  int selector_ = 0;
  std::atomic<size_t> currentTest_ = 0;

  std::shared_ptr<Test> nextTest() final;
  std::shared_ptr<Test> createTest(std::string const& name, std::string const& plugin) final;
  void onFinish() final;

};

class JobPage: public ComponentBase
{
  public:
  JobPage(std::string const& filePath);
  ~JobPage();

  private:
  const std::string buttonLabels_[4] = {" Run ", " ·   ", "  ·  ", "   · "};
  JobRunner runner_;
  std::shared_ptr<JobPanel> panel_;
  size_t ticker_ = 0;
  bool selectAll_ = false;
  // int switch_ = 0;
  std::atomic<bool> running_ = false;

  Component main_;

  bool OnEvent(Event event) final;
};
}