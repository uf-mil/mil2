#include "GripperControl.hh"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

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

    std::cout << "[GripperControl] ROS2 node created and subscribed to /keyboard/keypress" << std::endl;

    // Try to obtain model name (from Entity Name component or SDF attribute)
    auto nameComp = ecm.Component<gz::sim::components::Name>(entity);
    if (nameComp)
    {
        this->model_name_ = nameComp->Data();
        std::cout << "[GripperControl] Got model name from Entity: " << this->model_name_ << std::endl;
    }
    else if (sdf && sdf->HasAttribute("name"))
    {
        try
        {
            this->model_name_ = sdf->Get<std::string>("name");
            std::cout << "[GripperControl] Got model name from SDF: " << this->model_name_ << std::endl;
        }
        catch (std::exception const &e)
        {
            gzerr << "[GripperControl] Failed to get model name from SDF attribute 'name': " << e.what() << std::endl;
        }
    }

    // Parse SDF parameters for joint name and positions (if provided)
    if (sdf)
    {
        std::cout << "[GripperControl] Parsing SDF parameters ..." << std::endl;

        if (sdf->HasElement("joint_name"))
        {
            this->joint_name_ = sdf->Get<std::string>("joint_name");
        }
        // Optional explicit left/right joint names
        if (sdf->HasElement("left_joint_name"))
        {
            this->left_joint_name_ = sdf->Get<std::string>("left_joint_name");
        }
        if (sdf->HasElement("right_joint_name"))
        {
            this->right_joint_name_ = sdf->Get<std::string>("right_joint_name");
        }
        if (sdf->HasElement("open_pos"))
        {
            this->open_pos_ = sdf->Get<double>("open_pos");
        }
        if (sdf->HasElement("closed_pos"))
        {
            this->closed_pos_ = sdf->Get<double>("closed_pos");
        }
    }

    // Determine world name (for logging) by iterating Worlds
    // ecm.Each<gz::sim::components::World, gz::sim::components::Name>(
    //     [&](gz::sim::Entity const & /*ent*/, gz::sim::components::World const *,
    //         gz::sim::components::Name const *name) -> bool
    //     {
    //         this->world_name_ = name->Data();
    //         return false;  // stop after first world found
    //     });

    // std::cout << "[GripperControl] World Name: " << this->world_name_ << std::endl;

    // Build Gazebo joint command topic and advertise
    if (!this->model_name_.empty())
    {
        // topic pattern: /model/<model_name>/joint/<joint_name>/cmd_pos
        // Compute left/right topic names and advertise both
        std::string left_topic = "/model/" + this->model_name_ + "/joint/" + this->left_joint_name_ + "/cmd_pos";
        std::string right_topic = "/model/" + this->model_name_ + "/joint/" + this->right_joint_name_ + "/cmd_pos";

        std::cout << "[GripperControl] Advertising gz topics: " << left_topic << " , " << right_topic << std::endl;
        this->gz_pub_left_ = this->gz_node_.Advertise<gz::msgs::Double>(left_topic);
        this->gz_pub_right_ = this->gz_node_.Advertise<gz::msgs::Double>(right_topic);

        if (!this->gz_pub_left_)
        {
            gzerr << "[GripperControl] Failed to create gz publisher for topic: " << left_topic << std::endl;
        }
        if (!this->gz_pub_right_)
        {
            gzerr << "[GripperControl] Failed to create gz publisher for topic: " << right_topic << std::endl;
        }
    }
    else
    {
        gzerr << "[GripperControl] Topic name left empty because model name is unknown." << std::endl;
    }

    // Find joint entities by name so we can directly set joint positions
    ecm.Each<gz::sim::components::Name>(
        [&](gz::sim::Entity const &_ent, gz::sim::components::Name const *_name) -> bool
        {
            if (_name && _name->Data() == this->left_joint_name_)
            {
                this->left_joint_entity_ = _ent;
                std::cout << "[GripperControl] Found left joint entity: " << this->left_joint_name_ << std::endl;
            }
            if (_name && _name->Data() == this->right_joint_name_)
            {
                this->right_joint_entity_ = _ent;
                std::cout << "[GripperControl] Found right joint entity: " << this->right_joint_name_ << std::endl;
            }
            return true;  // continue iterating
        });
}

void GripperControl::KeypressCallback(std_msgs::msg::String::SharedPtr const msg)
{
    if (!msg)
        return;

    std::cout << "[GripperControl] Keypress received: " << msg->data << std::endl;

    // Toggle only on 'u' press (one-shot)
    if (msg->data == "u" && !this->u_pressed_)
    {
        this->u_pressed_ = true;
    }
}

void GripperControl::PreUpdate(gz::sim::UpdateInfo const & /*info*/, gz::sim::EntityComponentManager &_ecm)
{
    // If a 'u' press was detected (set in ROS callback), toggle and apply immediately
    if (this->u_pressed_)
    {
        this->gripper_open_ = !this->gripper_open_;
        double cmd = this->gripper_open_ ? this->open_pos_ : this->closed_pos_;

        std::vector<double> posData{ cmd };

        bool did_direct = false;
        if (this->left_joint_entity_ != gz::sim::kNullEntity)
        {
            gz::sim::Joint left(this->left_joint_entity_);
            left.ResetPosition(_ecm, posData);
            did_direct = true;
        }
        if (this->right_joint_entity_ != gz::sim::kNullEntity)
        {
            gz::sim::Joint right(this->right_joint_entity_);
            right.ResetPosition(_ecm, posData);
            did_direct = true;
        }

        if (!did_direct)
        {
            // Fallback to topic publishing if direct control not available
            gz::msgs::Double msg;
            msg.set_data(cmd);
            if (this->gz_pub_left_)
                this->gz_pub_left_.Publish(msg);
            if (this->gz_pub_right_)
                this->gz_pub_right_.Publish(msg);
            std::cout << "[GripperControl] Published cmd_pos = " << cmd << " to left/right gripper topics (fallback)"
                      << std::endl;
        }
        else
        {
            std::cout << "[GripperControl] Set joint positions directly (gripper_open="
                      << (this->gripper_open_ ? "true" : "false") << ")" << std::endl;
        }

        this->u_pressed_ = false;
    }
}

void GripperControl::PostUpdate(gz::sim::UpdateInfo const &info, gz::sim::EntityComponentManager const &ecm)
{
    if (info.paused)
        return;

    // Let ROS2 handle incoming messages for this node
    rclcpp::spin_some(this->node_);
    (void)ecm;  // PostUpdate is read-only; active control happens in PreUpdate
}

}  // namespace gripper_control

// Register plugin for Gazebo (include PreUpdate interface so PreUpdate() is called)
GZ_ADD_PLUGIN(gripper_control::GripperControl, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)
