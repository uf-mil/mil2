#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "tree_fixture.hpp"

// Every mission must at least load: node names, required ports, literal port
// values. This catches a typo or a CircleObject without a direction at build
// time instead of on the dock.
TEST_F(TreeFixture, EveryMissionLoads)
{
    auto &factory = prop_mission_planner::factory();
    factory.registerBehaviorTreeFromFile(std::string(MISSIONS_DIR) + "/prop_missions.xml");

    auto const ids = factory.registeredBehaviorTrees();
    for (char const *expected : { "FaceTest", "ApproachTest", "CircleTest", "BuoyTour" })
    {
        EXPECT_NE(std::find(ids.begin(), ids.end(), expected), ids.end()) << expected << " is not in the index";
    }
    for (auto const &id : ids)
    {
        EXPECT_NO_THROW((void)factory.createTree(id, blackboard_)) << id;
    }
}
