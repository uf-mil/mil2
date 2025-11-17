#pragma once

// Things this node expects from users:
// 1. this node expects its children to NEVER return IDLE when ticked
// 2. this node will stop ticking it's children once they return success or failure and YOU must tick this node again
// for it to start ticking it's children again In other words, tick this node once, and it will tick it's children to
// completion (then stop)

#include <behaviortree_cpp/decorator_node.h>

#include "context.hpp"

template <typename MsgT>
class TopicTicker : public BT::DecoratorNode
{
  public:
    TopicTicker(std::string const& name, const BT::NodeConfig& config) : BT::DecoratorNode(name, config)
    {
        get_port_data();

        // sub to thing
        sub_ = ctx_->node->create_subscription<MsgT>(topic_name_, 10, [this](MsgT const& msg) { this->topic_cb(msg); });
    }
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::shared_ptr<Context>>("ctx"), BT::InputPort<std::string>("topic_name") };
    }

  private:
    BT::NodeStatus childs_status_ = BT::NodeStatus::IDLE;
    std::shared_ptr<Context> ctx_;
    std::shared_ptr<rclcpp::Subscription<MsgT>> sub_;
    std::string topic_name_;

    void get_port_data()
    {
        auto ctx_res = getInput<std::shared_ptr<Context>>("ctx");
        if (!ctx_res)
        {
            throw BT::RuntimeError("TopicTicker requires [ctx] input");
        }
        ctx_ = ctx_res.value();

        auto topic_name_res = getInput<std::string>("topic_name");
        if (!topic_name_res)
        {
            throw BT::RuntimeError("TopicTicker requires [topic_name] input");
        }
        topic_name_ = topic_name_res.value();
    }

  protected:
    BT::NodeStatus tick() override
    {
        // if idle go to running
        if (childs_status_ == BT::NodeStatus::IDLE)
        {
            childs_status_ = BT::NodeStatus::RUNNING;
        }

        // temp status exists so I can change childs_status_ but return what it held before the change
        auto temp_status = childs_status_;
        if (childs_status_ == BT::NodeStatus::SUCCESS || childs_status_ == BT::NodeStatus::FAILURE)
        {
            childs_status_ = BT::NodeStatus::IDLE;
        }

        return temp_status;
    }

    // custom halt needed since node has internal state
    void halt() override
    {
        BT::DecoratorNode::halt();
        childs_status_ = BT::NodeStatus::IDLE;
    }

    void topic_cb(MsgT const& /*_*/)
    {
        if (childs_status_ != BT::NodeStatus::IDLE)
        {
            childs_status_ = child_node_->executeTick();
        }
    }
};
