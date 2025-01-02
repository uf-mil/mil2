#pragma once
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>

namespace mil_tools::fs {
std::optional<std::string> home();
std::string expanduser(std::string const &path);
}; // namespace mil_tools::fs
