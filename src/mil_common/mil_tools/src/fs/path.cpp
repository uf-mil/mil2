#include "mil_tools/fs/path.hpp"

namespace mil_tools::fs {

std::optional<std::string> home() {
  const char *home = std::getenv("HOME");
  return home ? std::make_optional<std::string>(home) : std::nullopt;
}

std::string expanduser(std::string const &path) {
  if (path.size() < 1 || path[0] != '~')
    return path;
  auto home_opt = home();
  if (!home_opt) {
    throw std::invalid_argument("$HOME is not set!");
  }

  return *home_opt + std::string(path).substr(1);
}

} // namespace mil_tools::fs