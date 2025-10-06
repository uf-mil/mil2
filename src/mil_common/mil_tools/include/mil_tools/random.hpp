#pragma once

#include <random>

namespace mil::random
{

std::string random_string(size_t length)
{
    std::string str;
    static std::string const ALPHANUM = "0123456789"
                                        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                        "abcdefghijklmnopqrstuvwxyz";
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, ALPHANUM.size() - 1);
    str.clear();
    str.reserve(length);
    for (size_t i = 0; i < length; ++i)
    {
        str.push_back(ALPHANUM[dis(gen)]);
    }
    return str;
}

}  // namespace mil::random
