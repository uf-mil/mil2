#pragma once

#include <behaviortree_cpp/decorator_node.h>

#include <cmath>
#include <vector>

// Sweeps the field ahead of the sub in a lawnmower pattern: advance `step`,
// slide to one edge, advance, slide to the other edge, until `length` is covered.
// The child must be a body-relative SendMove reading the `x`/`y` ports.
class LawnmowerSurvey final : public BT::DecoratorNode
{
  public:
    LawnmowerSurvey(std::string const& name, BT::NodeConfig const& cfg) : BT::DecoratorNode(name, cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("step", 1.0, "Forward distance between sweeps (m)"),
            BT::InputPort<double>("length", "Field length ahead of the sub (m)"),
            BT::InputPort<double>("width", "Field width, centred on the sub (m)"),
            BT::OutputPort<double>("x", "Current leg: metres forward"),
            BT::OutputPort<double>("y", "Current leg: metres left"),
        };
    }

  private:
    struct Leg
    {
        double x, y;
    };
    std::vector<Leg> legs_;
    size_t next_{ 0 };

    void plan()
    {
        double step = 1.0, length = 0.0, width = 0.0;
        getInput("step", step);
        getInput("length", length);
        getInput("width", width);

        legs_.clear();
        next_ = 0;

        double edge = width / 2;
        double at = 0.0;

        for (int i = std::floor(length / step + 1e-6); i > 0; --i)
        {
            legs_.push_back({ step, 0.0 });
            legs_.push_back({ 0.0, edge - at });
            at = edge;
            edge = -edge;
        }
    }

    BT::NodeStatus tick() override
    {
        if (status() == BT::NodeStatus::IDLE)
        {
            plan();
        }
        while (next_ < legs_.size())
        {
            if (child_node_->status() == BT::NodeStatus::IDLE)
            {
                setOutput("x", legs_[next_].x);
                setOutput("y", legs_[next_].y);
            }
            auto const child_status = child_node_->executeTick();
            if (child_status == BT::NodeStatus::RUNNING)
            {
                return child_status;
            }
            resetChild();
            if (child_status == BT::NodeStatus::FAILURE)
            {
                return child_status;
            }
            ++next_;
        }
        return BT::NodeStatus::SUCCESS;
    }
};
