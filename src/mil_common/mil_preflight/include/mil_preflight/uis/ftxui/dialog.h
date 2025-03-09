#pragma once

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

namespace mil_preflight
{
using namespace ftxui;

class Dialog: public ComponentBase, public std::enable_shared_from_this<Dialog>
{
    public:
    Dialog(std::string title, std::string question, std::vector<std::string> options)
    {
        Components buttons;
        for(size_t i = 0; i < options.size(); i++)
        {
            buttons.push_back(Button(options[i], [=]{
                ComponentBase* parent = Parent();
                restoreAllChildren(parent);
                onClose(i);
            }, ButtonOption::Border()));
        }
        Component buttonsContainer = Container::Horizontal(buttons);

        Component closeButton = Button("X", [=]{
            ComponentBase* parent = Parent();
            restoreAllChildren(parent);
            onClose(-1);
        }, ButtonOption::Ascii());

        // Component titleBar = Container::Horizontal({Render([]{return text()})});

        Component dialogContainer = Container::Vertical({
            Container::Horizontal({Renderer([t=std::move(title)]{return text(t);}) |flex ,closeButton}), 
            Renderer([]{return separator(); }),
            Renderer([q=std::move(question)]{return paragraph(q) | flex; }),
            buttonsContainer | center});

        Add(dialogContainer | border);
    }

    ~Dialog()
    {

    }

    void show(ComponentBase* parent)
    {
        saveAllChildren(parent);
    }

    protected:

    virtual void onClose([[maybe_unused]]int index){}

    private:

    void saveAllChildren(ComponentBase* parent)
    {
        for(size_t i=0;i<parent->ChildCount();i++)
        {
            Add(parent->ChildAt(i));
        }
        parent->Add(shared_from_this());
    }

    void restoreAllChildren(ComponentBase* parent)
    {
        Detach();
        for(size_t i=1;i<children_.size();i++)
        {
            parent->Add(children_[i]);
        }
    }

    Element Render() final
    {
        return children_[0]->Render();
    }
};

}