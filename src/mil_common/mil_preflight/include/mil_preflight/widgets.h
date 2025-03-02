#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

#include <boost/json.hpp>

#include <atomic>
#include <stack>

#include "mil_preflight/job.h"

using namespace ftxui;

namespace mil_preflight
{

class ActionBox: public ComponentBase, public Action
{
    public:
    // friend class TestPage;
    ActionBox(boost::json::key_value_pair const& pair);
    ~ActionBox();

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

    Element Render() final;
    bool OnEvent(Event event) final;
    inline bool OnMouseEvent(Event event);
    bool Focusable() const final;

    void onStart() final;
    void onSuccess(std::string const& info) final;
    void onFail(std::string const& info) final;

};

class TestPage: public ComponentBase, public Test
{
  public:
  using History = std::pair<size_t, bool>;

  TestPage(boost::json::key_value_pair const& pair);
  ~TestPage();

  size_t actionCount() {return ChildAt(0)->ChildCount();}

  size_t isChecked();
  inline void saveAndCheck(std::stack<History>& historys);
  inline void saveAndUncheck(std::stack<History>& historys);
  inline void restore(std::stack<History>& historys);
  inline void check();
  inline void uncheck();

  private:
  int selector_ = 0;
  bool checked_ = 0;
  std::atomic<size_t> currentAction_ = 0;
  Box box_;

  std::shared_ptr<Action> nextAction() final;
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

class JobPanel: public ComponentBase, public Job, public std::enable_shared_from_this<Job>
{
  public:
  JobPanel(boost::json::value const& value);
  ~JobPanel();

  private:

  Component tabsContainer_;
  Component pagesContainer_;

  int selector_ = 0;
  bool selectAll_ = false;
  size_t currentTest_ = 0;
  std::atomic<bool> running_ = false;

  const std::string buttonLabels_[4] = {" Run ", " ·   ", "  ·  ", "   · "};
  JobRunner runner_;
  size_t ticker_ = 0;

  std::shared_ptr<Test> nextTest() final;
  void onFinish() final;

};
}