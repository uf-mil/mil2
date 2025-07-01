#pragma once

#include <future>
#include <memory>
#include <string>
#include <vector>

namespace mil_preflight
{
static constexpr char ETX = 0x03;
static constexpr char EOT = 0x04;
static constexpr char ACK = 0x06;
static constexpr char BEL = 0x07;
static constexpr char NCK = 0x15;
static constexpr char GS = 0x1D;
static constexpr char US = 0x1F;

class Action
{
  public:
    Action() {};
    virtual ~Action() {};

    virtual std::string const& getName() const = 0;
    virtual std::vector<std::string> const& getParameters() const = 0;
    virtual void onStart() = 0;
    virtual void onFinish(bool success, std::string&& summery) = 0;
    virtual std::shared_future<int> onQuestion(std::string&& question, std::vector<std::string>&& options) = 0;

    std::vector<std::string> stdouts;
    std::vector<std::string> stderrs;

  private:
};

class Test
{
  public:
    Test() {};
    virtual ~Test() {};

    virtual std::string const& getName() const = 0;
    virtual std::string const& getPlugin() const = 0;
    virtual std::shared_ptr<Action> nextAction() = 0;
    virtual void onFinish() = 0;
};

class Job
{
  public:
    Job() {};
    ~Job() {};

    virtual std::shared_ptr<Test> nextTest() = 0;
    virtual void onFinish() = 0;
};

}  // namespace mil_preflight
