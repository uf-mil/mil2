#include "mil_preflight/tui.h"

#include <functional>
#include <queue>
#include <sstream>
#include <stack>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

#include "mil_preflight/frontend.h"

using namespace ftxui;
extern ScreenInteractive screen;

namespace mil_preflight
{

struct ActionReport
{
    bool success;
    std::string summery;
    std::vector<std::string> stdouts;
    std::vector<std::string> stderrs;
};
using TestReport = std::unordered_map<std::string, ActionReport>;
using JobReport = std::unordered_map<std::string, TestReport>;

class Window : public ComponentBase, public std::enable_shared_from_this<Window>
{
  public:
    struct Option
    {
        std::string title;
    };

    Window(std::string const& title) : screen_(ScreenInteractive::Fullscreen())
    {
        Component close_button = Button("X", [=] { screen_.Exit(); }, ButtonOption::Ascii());

        head_ = Container::Horizontal({ Renderer([=] { return text(title); }) | center | flex, close_button });
    }

    ~Window()
    {
    }

    void show(Component content)
    {
        Component main = Container::Vertical({ head_, Renderer([] { return separator(); }), content | flex });
        Add(main | border);

        screen_.Loop(shared_from_this());
    }

    void close()
    {
        screen_.Exit();
    }

  protected:
  private:
    Component head_;
    ScreenInteractive screen_;
};

class Dialog : public Window
{
  public:
    Dialog(std::string const& title) : Window(title)
    {
    }

    ~Dialog()
    {
    }

    int show(Component content, std::vector<std::string> const& options)
    {
        int ret = -1;
        Components buttons;
        for (size_t i = 0; i < options.size(); i++)
        {
            buttons.push_back(Button(
                options[i],
                [this, &ret, i]
                {
                    ret = i;
                    close();
                },
                ButtonOption::Border()));
        }

        Component main = Container::Vertical(
            { content | flex, Renderer([] { return separator(); }), Container::Horizontal(buttons) | center });

        Window::show(main);
        return ret;
    }

  private:
};

class MessageBox : public Dialog
{
  public:
    MessageBox(std::string const& title) : Dialog(title)
    {
    }

    ~MessageBox()
    {
    }

    int show(std::string const& message, std::vector<std::string> const& options)
    {
        std::istringstream iss(message);
        std::string line;
        Elements paragraphs;
        while (std::getline(iss, line))
        {
            paragraphs.push_back(paragraph(line));
        }

        return Dialog::show(Renderer([&] { return vbox({ paragraphs }); }), options);
    }

  private:
    Option option_;
};

static TestReport test_report;
static JobReport job_report;
static std::queue<JobReport> job_report_queue;

static std::queue<std::shared_ptr<Dialog>> questionQueue;

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
    ActionBox(ActionBoxOption&& option) : option_(std::move(option))
    {
    }
    ~ActionBox() {};

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
    bool Focusable() const final
    {
        return true;
    };

    void onStart() final
    {
        state_ = State::RUNNING;
    };
    void onFinish(bool success, std::string&& summery) final;
    std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options) final;
};

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

void ActionBox::onFinish(bool success, std::string&& summery)
{
    state_ = success ? State::SUCCESS : State::FAILED;
    ActionReport action_report = { success, std::move(summery), std::move(stdouts), std::move(stderrs) };
    test_report.emplace(getName(), std::move(action_report));
    screen.PostEvent(Event::Character("ActionFinish"));
}

std::shared_future<int> ActionBox::onQuestion(std::string&& question, std::vector<std::string>&& options)
{
    std::shared_ptr<std::promise<int>> feedback = std::make_shared<std::promise<int>>();

    std::shared_ptr<MessageBox> dialog = std::make_shared<MessageBox>("Question");
    screen.Post(
        [=]
        {
            int index = dialog->show(question, options);
            feedback->set_value(index);
        });

    return feedback->get_future().share();
}

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

std::shared_ptr<Action> TestTab::nextAction()
{
    Component child = option_.childContainer;
    while (currentAction_ < child->ChildCount())
    {
        std::shared_ptr<ActionBox> action = std::dynamic_pointer_cast<ActionBox>(child->ChildAt(currentAction_++));
        child->SetActiveChild(action);
        if (action->isChecked())
            return action;

        action->reset();
    }

    return nullptr;
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

void TestTab::onFinish()
{
    currentAction_ = 0;
    job_report.emplace(getName(), std::move(test_report));
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

TestsPage::TestsPage(std::function<void(std::shared_ptr<TestsPage>)> onRun)
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
            onRun(shared_from_this());
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

std::shared_ptr<Test> TestsPage::nextTest()
{
    running_ = true;

    while (currentTest_ < tabsContainer_->ChildCount())
    {
        std::shared_ptr<TestTab> test = std::dynamic_pointer_cast<TestTab>(tabsContainer_->ChildAt(currentTest_++));
        if (test->isChecked())
        {
            tabsContainer_->SetActiveChild(tabsContainer_->ChildAt(currentTest_ - 1));
            return test;
        }
    }
    return nullptr;
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

void TestsPage::onFinish()
{
    running_ = false;
    currentTest_ = 0;
    job_report_queue.push(std::move(job_report));
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
    ActionReportPanel(ActionReport&& report) : report_(std::move(report))
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
    ActionReport&& report_;
    Elements summeries_;
    Elements stdouts_;
    Elements stderrs_;
};

class TestReportPanel : public ComponentBase
{
  public:
    TestReportPanel(std::string const& name, TestReport&& report, bool* errorOnly) : errorOnly_(errorOnly)
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
    JobReportPanel(JobReport&& report, bool* errorOnly) : report_(std::move(report)), errorOnly_(errorOnly)
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
    JobReport report_;
    Component left_;
    Component right_;
    int selector_ = 0;
    bool rendered_ = false;
    bool* errorOnly_;
    int mainSize_ = 20;
};

class ReportsPage : public ComponentBase
{
  public:
    ReportsPage()
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

    ~ReportsPage() {};

  private:
    JobReport report_;

    bool showSuccess_ = true;
    Component reportPanel_;
    Component bottom_;
    int selector_ = 0;

    bool OnEvent(Event event) final
    {
        if (reportPanel_->ChildCount() == 0)
            return false;

        return ComponentBase::OnEvent(event);
    }

    Element Render() final
    {
        while (job_report_queue.size() > 0)
        {
            report_ = std::move(job_report_queue.front());

            if (report_.size() != 0)
            {
                reportPanel_->Add(std::make_shared<JobReportPanel>(std::move(report_), &showSuccess_));
                if (reportPanel_->ChildCount() > 1)
                    selector_ += 1;
            }

            job_report_queue.pop();
        }

        if (reportPanel_->ChildCount() > 0)
            return vbox({
                reportPanel_->Render() | flex,
                separator(),
                bottom_->Render(),
            });

        return text("No report available, please run some tests first.") | center;
    }
};

class Root : public ComponentBase
{
  public:
    Root()
    {
        Component back_button = Button(
                                    "<", [this] { current_page = 0; }, ButtonOption::Ascii()) |
                                Maybe([this] { return current_page != 0; });
        Component head =
            Renderer(back_button,
                     [this, back_button]
                     {
                         return hbox({ back_button->Render(), text(titles[current_page]) | bold | center |
                                                                  flex });  // Back button & title in same line
                     });

        body = Container::Tab({}, &current_page);
        menu = Container::Vertical({});
        body->Add(menu | center);
        Add(Container::Vertical({ head, Renderer([] { return separator(); }), body | flex }));
    }

    ~Root()
    {
    }

    void addPage(std::string const menu_entry, Component page, std::string const& title)
    {
        ButtonOption menu_option = ButtonOption::Ascii();
        menu_option.transform = [](EntryState const& s)
        {
            std::string const t = s.active ? "> " + s.label + " ⋯"  //
                                             :
                                             "  " + s.label + " ⋯";
            return text(t) | (s.focused ? bold : nothing);
        };

        int index = body->ChildCount();
        menu->Add(Button(menu_entry, [this, index] { current_page = index; }, menu_option));
        body->Add(page);
        titles.push_back(title);
    }

    void addCallback(std::string const menu_entry, std::function<void(void)> action)
    {
        ButtonOption menu_option = ButtonOption::Ascii();
        menu_option.transform = [](EntryState const& s)
        {
            std::string const t = s.active ? "> " + s.label  //
                                             :
                                             "  " + s.label;
            return text(t) | (s.focused ? bold : nothing);
        };

        menu->Add(Button(menu_entry, action, menu_option));
    }

  private:
    int current_page = 0;
    std::vector<std::string> titles = { "MIL Preflight" };
    Component body;
    Component menu;
};

static std::optional<boost::property_tree::ptree> loadConfig(int argc, char* argv[])
{
    boost::filesystem::path filename;
    if (argc > 1)
    {
        filename = argv[1];
    }
    else
    {
        filename = boost::process::search_path("mil_preflight").parent_path();
        std::vector<std::string> filenames;
        int selected = 0;

        // Create the RadioBox component
        for (auto const& entry : boost::filesystem::directory_iterator(filename))
        {
            if (entry.is_regular_file() && entry.path().extension() == ".json")
            {
                filenames.push_back(entry.path().string());
            }
        }

        if (filenames.size() > 0)
        {
            auto content = Radiobox(&filenames, &selected);
            std::shared_ptr<Dialog> dialog = std::make_shared<Dialog>("Load config");
            if (dialog->show(content, { "Go" }) == -1)
                return std::nullopt;
        }
        else
        {
            std::shared_ptr<MessageBox> message_box = std::make_shared<MessageBox>("Load config");
            message_box->show("No config file found under\n" + filename.string(), { "Quit" });
            return std::nullopt;
        }
        filename = filenames[selected];
    }

    // Parse the configuration file
    boost::property_tree::ptree cfg;
    try
    {
        boost::property_tree::read_json(filename.string(), cfg);
    }
    catch (boost::property_tree::json_parser_error const& e)
    {
        std::shared_ptr<MessageBox> message_box = std::make_shared<MessageBox>("Load config");
        message_box->show("Failed to parse\n" + std::string(e.what()), { "Quit" });
        return std::nullopt;
    }

    return cfg;
}

static void createJob(boost::property_tree::ptree& cfg, std::shared_ptr<TestsPage> tests_page)
{
    for (auto& test_pair : cfg)
    {
        std::string test_name = std::move(test_pair.first);
        boost::property_tree::ptree& test_node = test_pair.second;

        std::shared_ptr<TestTab> test_tab =
            tests_page->createTest(std::move(test_name), test_node.get<std::string>("plugin"));

        if (!test_tab)
            continue;

        for (auto& action_pair : test_node.get_child("actions"))
        {
            std::string action_name = std::move(action_pair.first);
            boost::property_tree::ptree& params_array = action_pair.second;

            std::vector<std::string> parameters;
            for (auto& param : params_array)
            {
                parameters.push_back(std::move(param.second.get_value<std::string>()));
            }

            test_tab->createAction(std::move(action_name), std::move(parameters));
        }
    }
}

TUI::TUI(int argc, char* argv[])
{
    std::shared_ptr<mil_preflight::Frontend> frontend = std::make_shared<mil_preflight::Frontend>();
    std::optional<boost::property_tree::ptree> cfg = loadConfig(argc, argv);

    if (!cfg)
    {
        Add(Renderer(
            []
            {
                ScreenInteractive::Active()->Exit();
                return text("");
            }));
        return;
    }

    std::shared_ptr<Root> root = std::make_shared<Root>();

    auto tests_page = std::make_shared<mil_preflight::TestsPage>([this, frontend](std::shared_ptr<TestsPage> page)
                                                                 { frontend->runJobAsync(page); });

    createJob(cfg.value(), tests_page);

    Component reports_page = std::make_shared<mil_preflight::ReportsPage>();

    Component about_page = Renderer(
        [this]
        {
            return vbox({ text("MIL Preflight") | bold, text("v1.0.0"), text("University of Florida"),
                          hbox({ text("Machine Intelligence Laboratory") | hyperlink("https://mil.ufl.edu/") }),
                          hbox({ text("Powered by "), text("FTXUI") | hyperlink("https://github.com/ArthurSonzogni/"
                                                                                "FTXUI/") }),
                          separatorEmpty(),
                          paragraph("MIL Preflight is a tool inspired by the preflight checklists used by "
                                    "pilots before flying a plane."),
                          paragraph("This program is designed to verify the functionality of all software and "
                                    "hardware systems on your autonomous robot."),
                          paragraph("It ensures that everything is in working order, allowing you to safely "
                                    "deploy your robot with confidence.") }) |
                   flex;
        });

    root->addPage("Run Tests", tests_page, "Tests");
    root->addPage("View Reports", reports_page, "Reports");
    root->addPage("About", about_page, "About");
    root->addCallback("Quit", [&] { ScreenInteractive::Active()->Exit(); });

    Add(root);
}

TUI::~TUI()
{
}

}  // namespace mil_preflight
