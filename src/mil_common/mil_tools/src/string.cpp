#include "mil_tools/string.hpp"

namespace mil_tools::string
{
std::vector<std::string> split(std::string const &str, std::string const &delim)
{
  std::vector<std::string> result;
  std::string::size_type current = 0;
  std::string::size_type next = 0;
  while ((next = str.find(delim, current)) != std::string::npos)
  {
    result.push_back(str.substr(current, next - current));
    current = next + delim.size();
  }
  result.push_back(str.substr(current));
  return result;
}

bool startswith(std::string const &s, std::string const &prefix)
{
  if (prefix.size() > s.size())
  {
    return false;
  }
  return std::equal(prefix.begin(), prefix.end(), s.begin());
}

bool endswith(std::string const &s, std::string const &suffix)
{
  if (suffix.size() > s.size())
  {
    return false;
  }
  return std::equal(suffix.rbegin(), suffix.rend(), s.rbegin());
}

// from https://stackoverflow.com/a/217605
std::string lstrip(std::string const &s)
{
  std::string res = s;
  res.erase(res.begin(), std::find_if(res.begin(), res.end(), [](unsigned char ch) { return !std::isspace(ch); }));
  return res;
}

std::string rstrip(std::string const &s)
{
  std::string res = s;
  res.erase(std::find_if(res.rbegin(), res.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(),
            res.end());
  return res;
}

std::string strip(std::string const &s)
{
  return lstrip(rstrip(s));
}
}  // namespace mil_tools::string
