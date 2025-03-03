#ifndef DVL_SENSOR_HH_
#define DVL_SENSOR_HH_

#include <gz/sim/System.hh>

namespace dvl_sensor
{
class ExamplePlugin : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate
{
public:
  ExamplePlugin();

public:
  ~ExamplePlugin() override;

  /// \brief Configure the system
  /// \param[in] _entity The entity this plugin is attached to.
  /// \param[in] _sdf The SDF Element associated with this system plugin.
  /// \param[in] _ecm The EntityComponentManager of the given simulation instance.
  /// \param[in] _eventMgr The EventManager of the given simulation instance.
  void Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &eventMgr) override;

public:
  void PostUpdate(gz::sim::UpdateInfo const &_info, gz::sim::EntityComponentManager const &_ecm) override;
};

}  // namespace dvl_sensor

#endif
