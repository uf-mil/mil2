#ifndef position_HH_
#define position_HH_

#include <gz/msgs/pose.pb.h>

#include <string>

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace real_position
{
class RealPosition : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
  public:
    RealPosition();

  public:
    ~RealPosition() override;

    /// \brief Configure the system
    /// \param[in] _entity The entity this plugin is attached to.
    /// \param[in] _sdf The SDF Element associated with this system plugin.
    /// \param[in] _ecm The EntityComponentManager of the given simulation instance.
    /// \param[in] _eventMgr The EventManager of the given simulation instance.
    void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

  public:
    void PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm) override;

  private:
    /// \brief Transport node for publishing
    gz::transport::Node node;

    /// \brief Publisher for pose messages
    gz::transport::Node::Publisher posePub;

    /// \brief Entity ID that this plugin is attached to
    gz::sim::Entity entityId;

    /// \brief Topic name for publishing pose
    std::string topicName;

    /// \brief Entity name for logging
    std::string entityName;

    /// \brief Update frequency control
    int updateCounter = 0;
    int publishFrequency = 10;  // Publish every 10 updates (adjust as needed)
};

}  // namespace real_position

#endif
