#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <string>
#include <vector>
#include <mutex>
#include <thread>

namespace gz {
namespace sim {
inline namespace v8 {
namespace systems {

class Sub9VelocityReset : public System,
                          public ISystemConfigure,
                          public ISystemPreUpdate
{
public:
    // ... (Configure function remains the same as the last version) ...
  void Configure(const Entity & /*_entity*/,
                 const std::shared_ptr<const sdf::Element> & _sdf,
                 EntityComponentManager & /*_ecm*/,
                 EventManager & /*_eventMgr*/) override
  {
    if (_sdf->HasElement("model_name"))
      this->modelName_ = _sdf->Get<std::string>("model_name");

    this->linearGain_ = _sdf->Get<double>("linear_gain", 500.0).first;
    this->angularGain_ = _sdf->Get<double>("angular_gain", 500.0).first;

    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    this->node_ = rclcpp::Node::make_shared("sub9_velocity_reset_plugin");
    this->srv_ = this->node_->create_service<std_srvs::srv::Empty>(
      "~/reset_sub9_velocity",
      std::bind(&Sub9VelocityReset::OnReset, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->node_->get_logger(),
                "[Sub9VelocityReset] Initialized. Model='%s'. Gains L:%.1f A:%.1f",
                this->modelName_.c_str(), this->linearGain_, this->angularGain_);
  }

  void PreUpdate(const UpdateInfo & /*_info*/,
                 EntityComponentManager & _ecm) override
  {
    rclcpp::spin_some(this->node_);

    if (this->modelEntity_ == kNullEntity)
    {
      this->modelEntity_ = this->findModel(this->modelName_, _ecm);
      if (this->modelEntity_ == kNullEntity) return;
    }

    if (!this->linksCached_)
    {
      this->cacheLinks(_ecm);
      if (this->links_.empty()) return;
      this->linksCached_ = true;
      RCLCPP_INFO(this->node_->get_logger(), "[Sub9VelocityReset] Cached %zu link(s)", this->links_.size());
    }

    // --- THE FIX: This block runs once to enable velocity checks ---
    if (this->linksCached_ && !this->velocityChecksEnabled_)
    {
        for (const auto &linkEid : this->links_)
        {
            gz::sim::Link link(linkEid);
            link.EnableVelocityChecks(_ecm, true);
        }
        this->velocityChecksEnabled_ = true;
        RCLCPP_INFO(this->node_->get_logger(), "[Sub9VelocityReset] Enabled velocity checks for %zu link(s).", this->links_.size());
    }
    // --- END OF FIX ---

    std::lock_guard<std::mutex> lock(this->mutex_);
    if (!this->isBraking_)
      return;

    // The rest of the braking logic will now work correctly
    double totalSpeedSq = 0;
    for (const auto &linkEid : this->links_)
    {
        gz::sim::Link link(linkEid);
        auto linVelComp = _ecm.Component<components::LinearVelocity>(linkEid);
        if (!linVelComp) continue; // It's possible a link has no velocity, just skip it

        auto linVel = linVelComp->Data();
        // Only apply braking to links that are actually moving
        if (linVel.SquaredLength() > 1e-6)
        {
            auto angVelComp = _ecm.Component<components::AngularVelocity>(linkEid);
            auto angVel = angVelComp ? angVelComp->Data() : gz::math::Vector3d::Zero;
            link.AddWorldWrench(_ecm, -linVel * this->linearGain_, -angVel * this->angularGain_);
        }
        totalSpeedSq += linVel.SquaredLength();
    }

    const double stopThresholdSq = 0.05;
    if (totalSpeedSq < stopThresholdSq)
    {
      RCLCPP_INFO(this->node_->get_logger(), "Braking complete. totalSpeedSq (%.6f) < threshold (%.6f)", totalSpeedSq, stopThresholdSq);
      this->isBraking_ = false;
    }
  }

private:
  // --- OnReset, findModel, cacheLinks functions are unchanged ---
  void OnReset(const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->isBraking_ = true;
    RCLCPP_INFO(this->node_->get_logger(), "[Sub9VelocityReset] Braking requested");
  }

  Entity findModel(const std::string &name, EntityComponentManager &ecm)
  {
    Entity found = kNullEntity;
    ecm.Each<components::Name, components::Model>(
      [&](const Entity &e, const components::Name *n, const components::Model *) -> bool
      {
        if (n->Data() == name) {
          found = e;
          return false;
        }
        return true;
      });
    return found;
  }

  void cacheLinks(EntityComponentManager &ecm)
  {
    this->links_.clear();
    ecm.Each<components::Link, components::ParentEntity>(
      [&](const Entity &e, const components::Link *, const components::ParentEntity *p) -> bool
      {
        if (p->Data() == this->modelEntity_)
          this->links_.push_back(e);
        return true;
      });
  }

  // --- Member Variables: Add one new flag ---
  std::mutex mutex_;
  std::string modelName_{"sub9"};
  Entity modelEntity_{kNullEntity};
  std::vector<Entity> links_;
  bool linksCached_{false};
  bool velocityChecksEnabled_{false}; // <-- THE NEW FLAG
  bool isBraking_{false};
  double linearGain_{500.0};
  double angularGain_{500.0};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_;
};

}
}
}
}

GZ_ADD_PLUGIN(
  gz::sim::v8::systems::Sub9VelocityReset,
  gz::sim::System,
  gz::sim::v8::systems::Sub9VelocityReset::ISystemConfigure,
  gz::sim::v8::systems::Sub9VelocityReset::ISystemPreUpdate)