#include "actuate_servo.hpp"

BT::PortsList ActuateServo::providedPorts()
{
    BT::PortsList ports;

    // Input ports
    ports.insert(BT::InputPort<std::shared_ptr<Context>>("ctx", "Shared Context"));
    ports.insert(BT::InputPort<bool>("gripper_is_open", "Whether to open (true) or close (false) the gripper"));
    ports.insert(BT::InputPort<bool>("marble_is_drop", "Whether the marble is being dropped (true) or not (false)"));
    ports.insert(BT::InputPort<bool>("torp_is_launch", "Whether the torpedo is being launched (true) or not (false)"));
    ports.insert(BT::InputPort<int>("servo_id", "ID of the servo to actuate (e.g. 1 for gripper, 2 for marble drop, 3 "
                                                "for torpedo launcher)"));
    // Servo IDs:
    // 1. Gripper
    // 2. Marble Dropper
    // 3. Torpedo Launcher

    // Outputs ports
    ports.insert(BT::OutputPort<bool>("gripper_status"));  // The status of the servo acutation attempt

    return ports;
}

BT::NodeStatus ActuateServo::tick()
{
    // Get Context - return FAILURE if not available
    if (!ctx_ && (!getInput("ctx", ctx_) || !ctx_))
    {
        setOutput("status", "Failed to get Context");
        return BT::NodeStatus::FAILURE;
    }

    // Get Servo ID - return FAILURE if not available
    if (!getInput("servo_id", servo_id_))
    {
        setOutput("status", "Failed to get servo_id");
        return BT::NodeStatus::FAILURE;
    }

    // Choose what ports to read from based on servo ID
    switch (servo_id_)
    {
        case 1:  // Gripper
            if (!getInput("gripper_is_open", gripper_is_open_))
            {
                setOutput("status", "Failed to get gripper_is_open input");
                return BT::NodeStatus::FAILURE;
            }

            break;
        case 2:  // Marble Dropper
            if (!getInput("marble_is_drop", marble_is_drop_))
            {
                setOutput("status", "Failed to get marble_is_drop input");
                return BT::NodeStatus::FAILURE;
            }
            break;
        case 3:  // Torpedo Launcher
            if (!getInput("torp_is_launch", torp_is_launch_))
            {
                setOutput("status", "Failed to get torp_is_launch input");
                return BT::NodeStatus::FAILURE;
            }
            break;
        default:
            setOutput("status", "Invalid servo_id input");
            return BT::NodeStatus::FAILURE;
    }

    //

    // Call actuateServo function and set status output based on result
    if (actuateServo(ctx_->node, ctx_->gripper_client, is_open_))
    {
        setOutput("status", is_open_ ? true : false);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        setOutput("status", is_open_ ? true : false);
        return BT::NodeStatus::FAILURE;
    }
}
