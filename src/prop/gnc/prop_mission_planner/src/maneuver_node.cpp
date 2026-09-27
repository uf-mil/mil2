#include "prop_mission_planner/maneuver_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include "prop_mission_planner/ports.hpp"

namespace prop_mission_planner
{

ManeuverNode::ManeuverNode(std::string const &name, BT::NodeConfig const &config) : BT::StatefulActionNode(name, config)
{
    // Exactly one of target / in_front. Checked here when the XML makes it
    // knowable (in_front literal or absent), so a wrong mission fails to load.
    // onStart checks again for the {blackboard} case.
    std::optional<bool> in_front;
    auto const it = config.input_ports.find("in_front");
    if (it == config.input_ports.end())
    {
        in_front = false;
    }
    else if (!BT::TreeNode::isBlackboardPointer(it->second))
    {
        in_front = BT::convertFromString<bool>(it->second);
    }
    if (in_front && port_given(config, "target") == *in_front)
    {
        throw BT::RuntimeError(name, ": give exactly one of target=\"{ref}\" or in_front=\"true\"");
    }
}

BT::PortsList ManeuverNode::common_ports()
{
    return {
        BT::InputPort<ObjectRef>("target", "The object to act on: a reference from StaticObject"),
        BT::InputPort<bool>("in_front", false, "Act on the nearest object ahead of the boat instead of a target"),
        BT::InputPort<double>("lock_timeout", 10.0, "Seconds (ROS time) to wait for a lock before failing"),
        BT::OutputPort<ObjectRef>("ref", "What was actually locked onto, for later steps"),
    };
}

BT::NodeStatus ManeuverNode::onStart()
{
    ctx_ = context_of(*this);
    maneuver_.reset();
    // A failed acquire keeps the previous lock, so start from nothing or this
    // node could carry on with the last maneuver's object.
    ctx_->maneuvers->lock.release();

    in_front_ = getInput<bool>("in_front").value_or(false);
    bool const has_target = port_given(config(), "target");
    if (has_target == in_front_)
    {
        RCLCPP_ERROR(ctx_->logger(), "%s: give exactly one of target=\"{ref}\" or in_front=\"true\"", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
    target_.reset();
    if (has_target)
    {
        auto const target = getInput<ObjectRef>("target");
        if (!target)
        {
            RCLCPP_ERROR(ctx_->logger(), "%s: target: %s", name().c_str(), target.error().c_str());
            return BT::NodeStatus::FAILURE;
        }
        target_ = *target;
    }

    lock_timeout_ = getInput<double>("lock_timeout").value_or(10.0);
    lock_budget_.arm(ctx_->node->now().nanoseconds(), static_cast<int>(lock_timeout_ * 1000.0));
    return try_lock();
}

BT::NodeStatus ManeuverNode::onRunning()
{
    return maneuver_ ? step() : try_lock();
}

void ManeuverNode::onHalted()
{
    if (ctx_)
    {
        stop_all();
    }
}

BT::NodeStatus ManeuverNode::try_lock()
{
    auto &maneuvers = *ctx_->maneuvers;
    auto &lock = maneuvers.lock;
    prop_maneuvers::Boat const boat = maneuvers.boat();

    bool locked = false;
    std::string why = "no position estimate yet";
    if (boat.valid)
    {
        locked = in_front_ ? lock.acquire_in_front(boat.position, boat.direction) : lock.acquire_near(target_->point);
        why = lock.why();
    }

    if (!locked)
    {
        if (lock_budget_.expired(ctx_->node->now().nanoseconds()))
        {
            RCLCPP_ERROR(ctx_->logger(), "%s: could not lock on within %.1f s: %s", name().c_str(), lock_timeout_,
                         why.c_str());
            stop_all();
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO_THROTTLE(ctx_->logger(), *ctx_->node->get_clock(), 2000, "%s: waiting to lock on: %s",
                             name().c_str(), why.c_str());
        return BT::NodeStatus::RUNNING;
    }

    lock_budget_.disarm();
    maneuver_ = make(maneuvers);
    if (config().output_ports.count("ref") > 0)
    {
        (void)setOutput("ref", ObjectRef{ lock.point() });
    }
    return step();
}

BT::NodeStatus ManeuverNode::step()
{
    switch (maneuver_->step())
    {
        case prop_maneuvers::Status::Running:
            return BT::NodeStatus::RUNNING;
        case prop_maneuvers::Status::Succeeded:
            // Leave everything as the maneuver left it, exactly like the
            // standalone programs, so behaviour matches PR #580.
            maneuver_.reset();
            return BT::NodeStatus::SUCCESS;
        case prop_maneuvers::Status::Failed:
            stop_all();
            return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::FAILURE;
}

void ManeuverNode::stop_all()
{
    ctx_->stop_motors();
    ctx_->maneuvers->lock.release();
    maneuver_.reset();
    lock_budget_.disarm();
}

}  // namespace prop_mission_planner
