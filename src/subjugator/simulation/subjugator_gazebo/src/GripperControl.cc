#include "GripperControl.hh"

#include <cmath>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>

namespace gripper_control
{

GripperControl::GripperControl()
{
}

GripperControl::~GripperControl()
{
}

void GripperControl::Configure(gz::sim::Entity const &entity, std::shared_ptr<sdf::Element const> const &sdf,
                               gz::sim::EntityComponentManager &ecm, gz::sim::EventManager & /*eventMgr*/)
{
    std::cout << "[GripperControl] Configuring GripperControl Plugin ..." << std::endl;

    // Initialize ROS2 if not already running
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // Create a ROS2 node for this plugin and subscribe to keyboard keypresses
    this->node_ = std::make_shared<rclcpp::Node>("gripper_control_plugin_node");
    this->key_sub_ = this->node_->create_subscription<std_msgs::msg::String>(
        "/keyboard/keypress", 10, std::bind(&GripperControl::KeypressCallback, this, std::placeholders::_1));

    this->gripper_srv_ = this->node_->create_service<subjugator_msgs::srv::Servo>(
        "gripper", std::bind(&GripperControl::GripperCallback, this, std::placeholders::_1, std::placeholders::_2));
    std::cout << "[GripperControl] Advertised Servo service '/gripper'" << std::endl;

    std::cout << "[GripperControl] ROS2 node created and subscribed to /keyboard/keypress" << std::endl;

    // Try to obtain model name (from Entity Name component or SDF attribute)
    auto nameComp = ecm.Component<gz::sim::components::Name>(entity);
    if (nameComp)
    {
        this->model_name_ = nameComp->Data();
        std::cout << "[GripperControl] Got model name from Entity: " << this->model_name_ << std::endl;
    }
    else
    {
        gzerr << "[GripperControl] Failed to get model name from Entity. Ensure the model has a Name component."
              << std::endl;
    }

    // Parse SDF parameters for joint name and positions (if provided)
    if (sdf)
    {
        std::cout << "[GripperControl] Parsing SDF parameters ..." << std::endl;

        // Grab Joint names from SDF
        if (sdf->HasElement("left_joint_name"))
        {
            this->left_joint_name_ = sdf->Get<std::string>("left_joint_name");
            std::cout << "[GripperControl] Left Joint Name acquired" << std::endl;
        }
        if (sdf->HasElement("right_joint_name"))
        {
            this->right_joint_name_ = sdf->Get<std::string>("right_joint_name");
            std::cout << "[GripperControl] Right Joint Name acquired" << std::endl;
        }

        // Proximity grasp parameters
        if (sdf->HasElement("attach_radius"))
        {
            this->attach_radius_ = sdf->Get<double>("attach_radius");
        }
        if (sdf->HasElement("gripper_link_name"))
        {
            this->gripper_link_name_ = sdf->Get<std::string>("gripper_link_name");
        }
        for (auto el = sdf->FindElement("graspable"); el; el = el->GetNextElement("graspable"))
        {
            this->graspable_names_.insert(el->Get<std::string>());
        }
        std::cout << "[GripperControl] " << this->graspable_names_.size()
                  << " graspable models, attach_radius=" << this->attach_radius_ << std::endl;
    }

    // Find joint entities by name so we can directly set joint positions
    ecm.Each<gz::sim::components::Name>(
        [&](gz::sim::Entity const &_ent, gz::sim::components::Name const *_name) -> bool
        {
            if (_name && _name->Data() == this->left_joint_name_)
            {
                this->left_joint_entity_ = _ent;
                std::cout << "[GripperControl] Found left joint entity: " << this->left_joint_name_ << std::endl;

                // Try to initialize current position from component if available
                auto posComp = ecm.Component<gz::sim::components::JointPosition>(this->left_joint_entity_);
                if (posComp && !posComp->Data().empty())
                {
                    this->left_current_pos_ = posComp->Data()[0];
                    this->left_target_pos_ = this->left_current_pos_;
                    this->left_pos_initialized_ = true;
                    std::cout << "[GripperControl] Left joint initial pos: " << this->left_current_pos_ << std::endl;
                }
            }
            if (_name && _name->Data() == this->gripper_link_name_)
            {
                this->gripper_link_entity_ = _ent;
                std::cout << "[GripperControl] Found gripper link entity: " << this->gripper_link_name_ << std::endl;
            }
            if (_name && _name->Data() == this->right_joint_name_)
            {
                this->right_joint_entity_ = _ent;
                std::cout << "[GripperControl] Found right joint entity: " << this->right_joint_name_ << std::endl;

                // Try to initialize current position from component if available
                auto posCompR = ecm.Component<gz::sim::components::JointPosition>(this->right_joint_entity_);
                if (posCompR && !posCompR->Data().empty())
                {
                    this->right_current_pos_ = posCompR->Data()[0];
                    this->right_target_pos_ = this->right_current_pos_;
                    this->right_pos_initialized_ = true;
                    std::cout << "[GripperControl] Right joint initial pos: " << this->right_current_pos_ << std::endl;
                }
            }
            return true;  // continue iterating
        });

    if (this->gripper_link_entity_ == gz::sim::kNullEntity)
    {
        std::cout << "[GripperControl] WARNING: gripper link '" << this->gripper_link_name_
                  << "' not found; proximity grasp disabled." << std::endl;
    }
    if (this->graspable_names_.empty())
    {
        std::cout << "[GripperControl] WARNING: no <graspable> models configured; "
                     "proximity grasp will never attach anything."
                  << std::endl;
    }
}

void GripperControl::KeypressCallback(std_msgs::msg::String::SharedPtr const msg)
{
    // If message is empty or invalid, ignore
    if (!msg)
        return;

    std::cout << "[GripperControl] Keypress received: " << msg->data << std::endl;

    // Toggle only on 'u' press (one-shot)
    if (msg->data == "u" && !this->u_pressed_)
    {
        this->u_pressed_ = true;
    }
}

void GripperControl::GripperCallback(std::shared_ptr<subjugator_msgs::srv::Servo::Request> const req,
                                     std::shared_ptr<subjugator_msgs::srv::Servo::Response> /*res*/)
{
    double frac = static_cast<double>(req->angle) / OPEN_ANGLE;
    frac = std::clamp(frac, 0.0, 1.0);
    double const tgt = this->closed_pos_ + frac * (this->open_pos_ - this->closed_pos_);

    // Service callback runs inside spin_some() in PostUpdate (gz thread), same
    // thread as PreUpdate -- safe to set targets directly.
    this->left_target_pos_ = tgt;
    this->right_target_pos_ = tgt;
    this->gripper_open_ = frac > 0.5;
    this->want_grasp_ = frac <= 0.5;  // closing => grasp intent

    std::cout << "[GripperControl] Servo cmd angle=" << static_cast<int>(req->angle) << " -> target=" << tgt
              << std::endl;
}

void GripperControl::PreUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager &_ecm)
{
    // If a 'u' press was detected (set in ROS callback), toggle and apply immediately
    if (this->u_pressed_)
    {
        // Set targets; actual motion will be smoothed over subsequent PreUpdate calls
        this->gripper_open_ = !this->gripper_open_;
        this->want_grasp_ = !this->gripper_open_;  // closed => grasp intent
        double tgt = this->gripper_open_ ? this->open_pos_ : this->closed_pos_;
        this->left_target_pos_ = tgt;
        this->right_target_pos_ = tgt;
        this->u_pressed_ = false;
    }

    // Perform smoothing toward targets each PreUpdate (exponential smoothing)
    double const eps = 1e-6;
    if (this->left_joint_entity_ != gz::sim::kNullEntity)
    {
        // ensure initialized
        if (!this->left_pos_initialized_)
        {
            auto posComp = _ecm.Component<gz::sim::components::JointPosition>(this->left_joint_entity_);
            if (posComp && !posComp->Data().empty())
            {
                this->left_current_pos_ = posComp->Data()[0];
            }
            this->left_target_pos_ = this->left_current_pos_;
            this->left_pos_initialized_ = true;
        }

        double delta = (this->left_target_pos_ - this->left_current_pos_) * this->smoothing_alpha_;
        this->left_current_pos_ += delta;
        if (std::fabs(this->left_target_pos_ - this->left_current_pos_) < 1e-4)
            this->left_current_pos_ = this->left_target_pos_;

        gz::sim::Joint left(this->left_joint_entity_);
        left.ResetPosition(_ecm, std::vector<double>{ this->left_current_pos_ });
    }

    if (this->right_joint_entity_ != gz::sim::kNullEntity)
    {
        if (!this->right_pos_initialized_)
        {
            auto posComp = _ecm.Component<gz::sim::components::JointPosition>(this->right_joint_entity_);
            if (posComp && !posComp->Data().empty())
            {
                this->right_current_pos_ = posComp->Data()[0];
            }
            this->right_target_pos_ = this->right_current_pos_;
            this->right_pos_initialized_ = true;
        }

        double delta = (this->right_target_pos_ - this->right_current_pos_) * this->smoothing_alpha_;
        this->right_current_pos_ += delta;
        if (std::fabs(this->right_target_pos_ - this->right_current_pos_) < 1e-4)
            this->right_current_pos_ = this->right_target_pos_;

        gz::sim::Joint right(this->right_joint_entity_);
        right.ResetPosition(_ecm, std::vector<double>{ this->right_current_pos_ });
    }

    // Edge-triggered grasp / release
    if (this->want_grasp_ && !this->grasp_active_)
    {
        this->TryGrasp(_ecm);
    }
    else if (!this->want_grasp_ && this->grasp_active_)
    {
        this->ReleaseGrasp();
    }

    // If the held model was removed from the sim, drop the grasp.
    if (this->grasp_active_ && this->held_model_entity_ != gz::sim::kNullEntity &&
        !_ecm.HasEntity(this->held_model_entity_))
    {
        this->ReleaseGrasp();
    }

    // While holding, keep the prop locked to the gripper (kinematic follow)
    if (this->grasp_active_ && this->held_model_entity_ != gz::sim::kNullEntity &&
        this->gripper_link_entity_ != gz::sim::kNullEntity)
    {
        gz::sim::Link link(this->gripper_link_entity_);
        auto linkPose = link.WorldPose(_ecm);
        if (linkPose)
        {
            gz::math::Pose3d const target = *linkPose * this->held_offset_;
            gz::sim::Model(this->held_model_entity_).SetWorldPoseCmd(_ecm, target);
        }
    }
}

void GripperControl::TryGrasp(gz::sim::EntityComponentManager &ecm)
{
    if (this->gripper_link_entity_ == gz::sim::kNullEntity)
        return;
    gz::sim::Link gripperLink(this->gripper_link_entity_);
    auto gpOpt = gripperLink.WorldPose(ecm);
    if (!gpOpt)
        return;
    gz::math::Pose3d const gripperPose = *gpOpt;

    gz::sim::Entity best = gz::sim::kNullEntity;
    double bestDist = this->attach_radius_;  // also serves as the radius cutoff (d < bestDist)
    ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](gz::sim::Entity const &ent, gz::sim::components::Model const *,
            gz::sim::components::Name const *name) -> bool
        {
            if (!name || this->graspable_names_.find(name->Data()) == this->graspable_names_.end())
                return true;
            gz::math::Pose3d const modelPose = gz::sim::worldPose(ent, ecm);
            double const d = (modelPose.Pos() - gripperPose.Pos()).Length();
            if (d < bestDist)
            {
                bestDist = d;
                best = ent;
            }
            return true;
        });

    if (best == gz::sim::kNullEntity)
        return;

    gz::math::Pose3d const modelPose = gz::sim::worldPose(best, ecm);
    this->held_model_entity_ = best;
    this->held_offset_ = gripperPose.Inverse() * modelPose;  // gripper_link -> model
    this->grasp_active_ = true;
    std::cout << "[GripperControl] Grasped model entity " << best << " at dist " << bestDist << std::endl;
}

void GripperControl::ReleaseGrasp()
{
    // Note: the prop's velocity is not zeroed here, so on release it keeps whatever
    // the physics engine last had. A follow-up could zero linear/angular velocity for
    // a cleaner drop if props fling on release.
    this->grasp_active_ = false;
    this->held_model_entity_ = gz::sim::kNullEntity;
    std::cout << "[GripperControl] Released grasp" << std::endl;
}

void GripperControl::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
    if (info.paused)
        return;

    // Let ROS2 handle incoming messages for this node
    rclcpp::spin_some(this->node_);
    (void)ecm;
}

}  // namespace gripper_control

// Register plugin for Gazebo (include PreUpdate interface so PreUpdate() is called)
GZ_ADD_PLUGIN(gripper_control::GripperControl, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
