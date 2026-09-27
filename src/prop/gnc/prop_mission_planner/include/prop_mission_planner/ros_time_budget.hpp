#pragma once

#include <cstdint>

// Pure deadline arithmetic for the ROS-time decorators (RosTimeout, RosDelay)
// and the maneuver nodes' lock timeout, unit-tested without ROS or BT.
// Timestamps are int64 nanoseconds from ctx->node->now().nanoseconds(): wall
// time on the boat, SIM time under use_sim_time -- which is the whole point.
// Adapted from the sub's mission planner (origin/gripper-task-5).
namespace ros_time_budget
{

struct Budget
{
    std::int64_t deadline_ns{ 0 };
    bool armed{ false };

    void arm(std::int64_t now_ns, int budget_msec)
    {
        deadline_ns = now_ns + static_cast<std::int64_t>(budget_msec) * 1000000LL;
        armed = true;
    }

    // Never true before arm(). >= so a zero budget expires on the very next
    // evaluation even when the clock has not advanced (e.g. sim paused).
    bool expired(std::int64_t now_ns) const
    {
        return armed && now_ns >= deadline_ns;
    }

    void disarm()
    {
        armed = false;
    }
};

}  // namespace ros_time_budget
