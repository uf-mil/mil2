#include "has_found_pair.hpp"

BT::PortsList HasFoundPair::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<bool>("found_pair", false, "Set true when a gate pair was found"));
    return ports;
}

BT::NodeStatus HasFoundPair::tick()
{
    bool fp = false;
    (void)getInput("found_pair", fp);
    return fp ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
