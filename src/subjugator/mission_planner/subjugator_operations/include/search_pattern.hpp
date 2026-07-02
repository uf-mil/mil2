#pragma once
#include <cmath>
#include <utility>
#include <vector>

namespace search_pattern
{

// Expanding square spiral of waypoints (offsets from the search center), spaced
// `step_m` apart, out to `max_radius` m. Pure geometry, unit-testable without ROS.
// Segment lengths grow 1,1,2,2,3,3,... turning +x -> +y -> -x -> -y.
inline std::vector<std::pair<double, double>> spiral_waypoints(double step_m, double max_radius)
{
    std::vector<std::pair<double, double>> pts;
    if (step_m <= 0.0 || max_radius <= 0.0)
        return pts;
    double x = 0.0, y = 0.0;
    int dir = 0;      // 0:+x 1:+y 2:-x 3:-y
    int seg_len = 1;  // grows every two turns
    int turns = 0;
    while (true)
    {
        for (int s = 0; s < seg_len; ++s)
        {
            switch (dir)
            {
                case 0:
                    x += step_m;
                    break;
                case 1:
                    y += step_m;
                    break;
                case 2:
                    x -= step_m;
                    break;
                default:
                    y -= step_m;
                    break;
            }
            if (std::hypot(x, y) > max_radius)
                return pts;
            pts.push_back({ x, y });
        }
        dir = (dir + 1) % 4;
        if (++turns % 2 == 0)
            ++seg_len;
    }
}

}  // namespace search_pattern
