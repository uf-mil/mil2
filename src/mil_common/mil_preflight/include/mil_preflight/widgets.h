#include <ftxui/component/component.hpp>

#include <boost/json.hpp>

#include "mil_preflight/job.h"

namespace mil_preflight
{
    using namespace ftxui;

    class ActionWidget: public Action
    {
        public:
        friend class TestWidget;
        ActionWidget() = delete;
        ActionWidget(boost::json::key_value_pair const& pair):
            impl(std::make_shared<ActionImpl>())
        {
            impl -> enable = false;
            impl -> name = pair.key();
            impl -> parameters = pair.value().as_string();
            impl -> state = State::SUCCESS;
            auto checkbox = Checkbox(impl->name, &impl->enable);
            component = Renderer(checkbox, [checkbox, impl = impl]{
                const char* stateInd;
                switch (impl->state)
                {
                case State::RUNNING:
                    stateInd = "-";
                    break;
                case State::SUCCESS:
                    stateInd = "✔";
                    break;
                case State::FAILED:
                    stateInd = "✘";
                    break;
                default:
                    stateInd =" ";
                    break;
                }

                return hbox({checkbox->Render() | flex ,text(stateInd) | align_right});
            });
        }
        ~ActionWidget()
        {

        }

        ActionWidget(ActionWidget&& action)
        {
            impl = std::move(action.impl);
            component = std::move(action.component);
        }

        ActionWidget& operator=(ActionWidget&& action) noexcept {
            if (this == &action) return *this; 

            impl = std::move(action.impl);
            component = std::move(action.component);

            return *this;
        }

        Component getComponent() const
        {
            return component;
        }

        private:

        enum class State
        {
            NONE,
            RUNNING,
            SUCCESS,
            FAILED
        };

        struct ActionImpl
        {
            std::string name;
            std::string parameters;
            bool enable;
            State state;
        };
        
        std::shared_ptr<ActionImpl> impl;
        Component component;
    };

    class TestWidget//: public Test
    {
        public:
        TestWidget(boost::json::key_value_pair const& pair): impl(std::make_shared<TestImpl>())
        {
            impl -> enable = false;
            impl -> name = pair.key();
            const boost::json::object& obj = pair.value().as_object();
            impl -> plugin = obj.at("plugin").as_string();

            Components comps;

            for(const auto& actionPair: obj.at("actions").as_object())
            {
                impl -> actions.push_back(ActionWidget(actionPair));
                comps.push_back(impl -> actions.back().getComponent());
            }

            container = Container::Vertical(comps) | vscroll_indicator | frame;
            checkbox = Checkbox(impl->name, &impl -> enable, 
                                CheckboxOption{.on_change=[impl=impl]{
                                    for(auto& action: impl->actions)
                                    {
                                        action.impl->enable = impl->enable;
                                    }
                                 }});
        }
        ~TestWidget()
        {
            
        }

        TestWidget(TestWidget&& test)
        {
            container = std::move(test.container);
            checkbox = std::move(test.checkbox);
            impl = std::move(test.impl);
        }

        TestWidget& operator=(TestWidget&& test) noexcept {
            if (this == &test) return *this; // Self-assignment check

            container = std::move(test.container);
            checkbox = std::move(test.checkbox);
            impl = std::move(test.impl);

            return *this;
        }

        // Action* nextAction()
        // {
        //     return 
        // }

        Component getContianer() const
        {
            return container;
        }

        Component getComponent() const
        {
            return checkbox;
        }

        private:

        struct TestImpl
        {
            std::string name;
            std::string plugin;
            std::vector<ActionWidget> actions;
            bool enable;

            // void onChange()
            // {
            //     for(auto& action: actions)
            //     {
            //         action.impl->enable = enable;
            //     }
            // }

        };

        Component container;
        Component checkbox;
        std::shared_ptr<TestImpl> impl;    
    };

    class JobWidget
    {
        public:
        JobWidget(boost::json::value const& value):
            impl(std::make_unique<JobImpl>()),
            render(std::bind(&JobWidget::onRender, this))
        {
            Components comps;
            Components conts;
            for(const auto& testPair: value.as_object())
            {
                impl -> tests.push_back(TestWidget(testPair));
                comps.push_back(impl -> tests.back().getComponent());
                conts.push_back(impl -> tests.back().getContianer());
            }

            auto right = Container::Tab(conts, &impl->currentPage);
            auto left = Container::Vertical(comps, &impl->selectedItem);
            auto runButton = Button("Run", [&] {}, ButtonOption::Border());

            container = Renderer(Container::Vertical({Container::Horizontal({left, right}), runButton}), [=]{
                return vbox({
                    hbox({
                        left->Render() | vscroll_indicator | frame,
                        separator(),  // 🔹 Vertical line between left & right panels
                        right->Render() | flex,
                    }) | flex,
                    separator(),
                    runButton->Render() | align_right
                });
            });
        }
        ~JobWidget()
        {

        }

        JobWidget(JobWidget&& job)
        {
            container = std::move(job.container);
            render = std::bind(&JobWidget::onRender, this);
            impl = std::move(job.impl);
        }

        JobWidget& operator= (JobWidget&& job)
        {
            if(this == &job) return *this;

            container = std::move(job.container);
            render = std::bind(&JobWidget::onRender, this);
            impl = std::move(job.impl);

            return *this;
        }

        Component getCompoent() const
        {
            return Renderer(container, render);
        }

        private:

        struct JobImpl
        {
            int currentPage;
            int selectedItem;
            std::vector<TestWidget> tests;
        };

        std::unique_ptr<JobImpl> impl;
        std::function<Element()> render;
        Component container;

        Element onRender()
        {
            impl->currentPage = impl->selectedItem;
            return container->Render();
        }
    };
}