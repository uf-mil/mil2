#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

// Decision-only leaf: maps (role, items_collected) to the correct octagon-wall
// symbol and writes it to `target_symbol`. role comes from the `role` port if
// non-empty, else from Context::get_role(). Pure logic in select_face_symbol_logic.hpp.
class SelectFaceSymbol : public BT::SyncActionNode
{
  public:
    SelectFaceSymbol(std::string const& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("role", "", "survey_repair|search_rescue; empty -> use Context role"),
                 BT::InputPort<int>("items_collected", 0, "Items collected so far (from S6)"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx"),
                 BT::OutputPort<std::string>("target_symbol", "Correct symbol to face ('' if none)") };
    }

    BT::NodeStatus tick() override;
};
