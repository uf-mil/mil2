#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "context.hpp"

class CheckYoloModel : public BT::SyncActionNode
{
  public:
    CheckYoloModel(std::string const& name, const BT::NodeConfiguration& cfg) : BT::SyncActionNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("model_filename", "new_sim_nav_channel.pt", "Model filename"),
                 BT::InputPort<std::string>("yolo_node", "/yolo/yolo_node", "yolo node name"),
                 BT::InputPort<std::string>("models_rel_dir", "models", "Relative dir in package share"),
                 BT::InputPort<std::shared_ptr<Context>>("ctx") };
    }

    BT::NodeStatus tick() override;

  private:
    bool ensureModelLoaded_(std::shared_ptr<Context> const& ctx, std::string const& node_name,
                            std::string const& full_path);
};
