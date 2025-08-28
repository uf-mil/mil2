#pragma once
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>

namespace mil::fs::path
{
std::optional<std::string> home();
std::string expanduser(std::string const &path);
};  // namespace mil::fs::path
