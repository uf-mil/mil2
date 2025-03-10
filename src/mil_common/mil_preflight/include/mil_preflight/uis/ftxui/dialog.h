#pragma once

#include <ftxui/component/component.hpp>
#include <ftxui/dom/elements.hpp>

namespace mil_preflight
{
using namespace ftxui;

class Dialog : public ComponentBase, public std::enable_shared_from_this<Dialog>
{
public:
  struct Option
  {
    std::string title;
    std::string question;
    std::vector<std::string> buttonLabels;
    std::function<void(int)> onClose = [](int) {};
  };

  Dialog(Option const& option) : option_(option)
  {
    create();
  }

  Dialog(Option&& option) : option_(std::move(option))
  {
    create();
  }

  ~Dialog()
  {
  }

  void show(ComponentBase* parent)
  {
    saveAllChildren(parent);
  }

  Element Render() override
  {
    return vbox({ hbox({ text(option_.title) | flex, closeButton_->Render() }), separator(),
                  paragraph(option_.question) | flex, separator(), buttonsContainer_->Render() | center }) |
           border;
  }

private:
  Component buttonsContainer_;
  Component closeButton_;
  Option option_;

  void create()
  {
    Components buttons;
    for (size_t i = 0; i < option_.buttonLabels.size(); i++)
    {
      buttons.push_back(Button(option_.buttonLabels[i], std::bind(&Dialog::onButton, this, i), ButtonOption::Border()));
    }
    buttonsContainer_ = Container::Horizontal(buttons);

    closeButton_ = Button("X", std::bind(&Dialog::onButton, this, -1), ButtonOption::Ascii());

    Component dialogContainer = Container::Vertical({ closeButton_, buttonsContainer_ });

    Add(dialogContainer);
  }

  void onButton(int index)
  {
    option_.onClose(index);
    ComponentBase* parent = Parent();
    restoreAllChildren(parent);
  }

  void saveAllChildren(ComponentBase* parent)
  {
    for (size_t i = 0; i < parent->ChildCount(); i++)
    {
      Add(parent->ChildAt(i));
    }
    parent->Add(shared_from_this());
  }

  void restoreAllChildren(ComponentBase* parent)
  {
    for (size_t i = 1; i < children_.size(); i++)
    {
      parent->Add(children_[i]);
    }
    Detach();
  }
};

}  // namespace mil_preflight
