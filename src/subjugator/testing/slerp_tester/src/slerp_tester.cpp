#include <array>
#include <cstdio>

#include "mil_tools/geometry/Rotation.hpp"
#include "mil_tools/geometry/Slerp.hpp"

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("hello world slerp_tester package\n");

    std::array<double, 4> one = { 0, 0, 0, 1 };  // w x y z
    std::array<double, 4> two = { 0, 0, 1, 0 };  // w x y z

    auto s = mil_tools::geometry::Slerp(one, two);

    std::cout << s.at(0) << std::endl;
    std::cout << s.at(0.1) << std::endl;
    std::cout << s.at(0.2) << std::endl;
    std::cout << s.at(0.3) << std::endl;
    std::cout << s.at(0.4) << std::endl;
    std::cout << s.at(0.5) << std::endl;
    std::cout << s.at(0.6) << std::endl;
    std::cout << s.at(0.7) << std::endl;
    std::cout << s.at(0.8) << std::endl;
    std::cout << s.at(0.9) << std::endl;
    std::cout << s.at(1.0) << std::endl;

    return 0;
}
