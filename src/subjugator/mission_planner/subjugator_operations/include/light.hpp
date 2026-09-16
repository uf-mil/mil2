#pragma once

#include <string>
#include <vector>

// A light logged during a survey, in the odom frame. Passed between nodes on the blackboard.
struct Light
{
    std::string color;
    double x, y;
};
using Lights = std::vector<Light>;
