#pragma once
#include <string>

struct MissionParams
{
    // Put mission-tunable params here if you want (timeouts, tolerances…)
};

class Mission
{
  public:
    virtual ~Mission() = default;
    virtual std::string id() const = 0;
    virtual std::string buildTreeXml(MissionParams const &) = 0;
};

class MovementMission : public Mission
{
  public:
    std::string id() const override
    {
        return "MovementMission";
    }
    std::string buildTreeXml(MissionParams const &) override;
};
