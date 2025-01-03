#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

namespace mil_tools::string {
std::vector<std::string> split(std::string const &s, std::string const &delim);
bool startswith(std::string const &s, std::string const &prefix);
bool endswith(std::string const &s, std::string const &suffix);
std::string strip(std::string const &s);
std::string lstrip(std::string const &s);
std::string rstrip(std::string const &s);
} // namespace mil_tools::string
