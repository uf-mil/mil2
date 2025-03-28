#ifndef ACTIVE_SONAR_HH_
#define ACTIVE_SONAR_HH_

#include <gz/sim/System.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>

// NOTE: .msg imports must be snake_case even though the file is in PascalCase
#include "mil_msgs/msg/echo_intensities.hpp"

namespace active_sonar
{
  class ActiveSonar:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
  {
    public: 
      ActiveSonar();
      ~ActiveSonar() override;
 
      /// \brief Configure the system
      /// \param[in] _entity The entity this plugin is attached to.
      /// \param[in] _sdf The SDF Element associated with this system plugin.
      /// \param[in] _ecm The EntityComponentManager of the given simulation instance.
      /// \param[in] _eventMgr The EventManager of the given simulation instance.
      void Configure(
      const gz::sim::Entity & entity, 
      const std::shared_ptr<const sdf::Element> & sdf,
      gz::sim::EntityComponentManager & ecm,
      gz::sim::EventManager & eventMgr) override;


      void PostUpdate(const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override;

      void receiveGazeboCallback(const gz::msgs::PointCloudPacked &msg);

    private: 
      gz::transport::Node node;
      gz::transport::Node::Publisher publisher;
  };
}

#endif