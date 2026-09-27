#include "prop_mission_planner/maneuver_node.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "prop_mission_planner/ports.hpp"

namespace prop_mission_planner
{

namespace
{
/// A usable lock timeout: finite and not negative.
bool valid_lock_timeout(double seconds)
{
    return std::isfinite(seconds) && seconds >= 0.0;
}
}  // namespace

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

    if (auto const literal = literal_port(config, "lock_timeout"))
    {
        double const seconds = BT::convertFromString<double>(*literal);
        if (!valid_lock_timeout(seconds))
        {
            throw BT::RuntimeError(name, ": lock_timeout must be a finite number of seconds >= 0, not \"", *literal,
                                   "\"");
        }
    }
}

ManeuverNode::~ManeuverNode()
{
    // A tree destroyed without being halted must not leave a dangling claim
    // that blocks (and is dereferenced by) the next maneuver node.
    if (ctx_ && ctx_->active_maneuver == this)
    {
        ctx_->active_maneuver = nullptr;
    }
}

BT::PortsList ManeuverNode::common_ports()
{
    return {
        BT::InputPort<ObjectRef>("target", "The object to act on: a reference from StaticObject"),
        BT::InputPort<bool>("in_front", false, "Act on the nearest object ahead of the boat instead of a target"),
        BT::InputPort<double>("lock_timeout", 10.0, "Seconds (ROS time) to wait for a lock before failing"),
        BT::OutputPort<ObjectRef>("ref", "Point locked onto (at lock time), for later steps"),
    };
}

BT::NodeStatus ManeuverNode::onStart()
{
    // context_of() throws if this tree was built without a Context on the
    // root blackboard. The header promises nothing throws out of a running
    // node, so catch it here: ctx_ stays null, and onHalted already guards on
    // that (it only calls stop_all() when ctx_ is set).
    try
    {
        ctx_ = context_of(*this);
    }
    catch (std::exception const &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("prop_mission_planner"), "%s: no Context on the tree: %s", name().c_str(),
                     e.what());
        return BT::NodeStatus::FAILURE;
    }
    maneuver_.reset();
    // Before touching the shared lock: releasing it here would pull the lock
    // out from under the maneuver that is driving.
    if (someone_else_driving())
    {
        return BT::NodeStatus::FAILURE;
    }
    // A failed acquire keeps the previous lock, so start from nothing or this
    // node could carry on with the last maneuver's object.
    ctx_->maneuvers->lock.release();

    auto const in_front = getInput<bool>("in_front");
    if (!in_front)
    {
        RCLCPP_ERROR(ctx_->logger(), "%s: in_front: %s", name().c_str(), in_front.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    in_front_ = *in_front;
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

    auto const lock_timeout = getInput<double>("lock_timeout");
    if (!lock_timeout || !valid_lock_timeout(*lock_timeout))
    {
        RCLCPP_ERROR(ctx_->logger(), "%s: lock_timeout must be a finite number of seconds >= 0 (%s)", name().c_str(),
                     lock_timeout ? std::to_string(*lock_timeout).c_str() : lock_timeout.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    lock_timeout_ = *lock_timeout;
    // Clamp before converting: a huge timeout would overflow int milliseconds.
    double const budget_msec = std::min(lock_timeout_ * 1000.0, static_cast<double>(INT_MAX));
    lock_budget_.arm(ctx_->node->now().nanoseconds(), static_cast<int>(budget_msec));
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

bool ManeuverNode::someone_else_driving() const
{
    ManeuverNode const *const other = ctx_->active_maneuver;
    if (other == nullptr || other == this)
    {
        return false;
    }
    RCLCPP_ERROR(ctx_->logger(), "%s: another maneuver (%s) is already driving", name().c_str(), other->name().c_str());
    return true;
}

BT::NodeStatus ManeuverNode::try_lock()
{
    // Checked on every attempt, not only in onStart: acquiring re-aims the
    // shared lock, which must not happen while another node is driving on it.
    // No stop_all() here -- the motors and the lock belong to the other node.
    if (someone_else_driving())
    {
        maneuver_.reset();
        lock_budget_.disarm();
        return BT::NodeStatus::FAILURE;
    }

    auto &maneuvers = *ctx_->maneuvers;
    auto &lock = maneuvers.lock;
    prop_maneuvers::Boat const boat = maneuvers.boat();

    bool locked = false;
    std::string why = "no position estimate yet";
    if (boat.valid)
    {
        // TODO(map/staleness): TargetLock::blobs() never ages out the last
        // cluster frame, and prop_maneuvers::Boat::valid never goes stale, so
        // a re-lock after clustering or the EKF stalls can use old data. The
        // standalone programs rarely hit this because they lock once.
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
    ctx_->active_maneuver = this;
    // Everything between taking the claim and the first step is guarded:
    // make() may throw, and so may setOutput (BT's Blackboard::set throws on
    // a type clash with an existing entry). Either way the claim must not be
    // left set, so stop_all() drops it.
    try
    {
        maneuver_ = make(maneuvers);
        if (!maneuver_)
        {
            RCLCPP_ERROR(ctx_->logger(), "%s: could not start the maneuver (see above)", name().c_str());
            stop_all();
            return BT::NodeStatus::FAILURE;
        }
        if (config().output_ports.count("ref") > 0)
        {
            (void)setOutput("ref", ObjectRef{ lock.point() });
        }
    }
    catch (std::exception const &e)
    {
        RCLCPP_ERROR(ctx_->logger(), "%s: could not start the maneuver: %s", name().c_str(), e.what());
        stop_all();
        return BT::NodeStatus::FAILURE;
    }
    return step();
}

BT::NodeStatus ManeuverNode::step()
{
    prop_maneuvers::Status status = prop_maneuvers::Status::Failed;
    try
    {
        status = maneuver_->step();
    }
    catch (std::exception const &e)
    {
        RCLCPP_ERROR(ctx_->logger(), "%s: maneuver failed: %s", name().c_str(), e.what());
        stop_all();
        return BT::NodeStatus::FAILURE;
    }

    switch (status)
    {
        case prop_maneuvers::Status::Running:
            return BT::NodeStatus::RUNNING;
        case prop_maneuvers::Status::Succeeded:
            // Leave everything as the maneuver left it, exactly like the
            // standalone programs, so behaviour matches PR #580. Only the
            // claim goes: the next maneuver node may now drive.
            maneuver_.reset();
            if (ctx_->active_maneuver == this)
            {
                ctx_->active_maneuver = nullptr;
            }
            return BT::NodeStatus::SUCCESS;
        case prop_maneuvers::Status::Failed:
            stop_all();
            return BT::NodeStatus::FAILURE;
    }
    stop_all();
    return BT::NodeStatus::FAILURE;
}

void ManeuverNode::stop_all()
{
    maneuver_.reset();
    lock_budget_.disarm();
    // The motors and the lock belong to whichever node is driving. If that is
    // another node (this one was only waiting), leave them alone.
    if (ctx_->active_maneuver != nullptr && ctx_->active_maneuver != this)
    {
        return;
    }
    ctx_->stop_motors();
    ctx_->maneuvers->lock.release();
    ctx_->active_maneuver = nullptr;
}

}  // namespace prop_mission_planner
