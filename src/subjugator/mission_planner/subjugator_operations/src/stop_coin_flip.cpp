#include "stop_coin_flip.hpp"

#include "start_coin_flip.hpp"  // stopCoinFlip()

BT::NodeStatus StopCoinFlip::tick()
{
    std::shared_ptr<Context> ctx;
    if (!getInput("ctx", ctx) || !ctx)
        return BT::NodeStatus::FAILURE;

    stopCoinFlip(*ctx);
    return BT::NodeStatus::SUCCESS;
}
