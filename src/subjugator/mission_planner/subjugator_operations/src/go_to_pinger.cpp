#include "go_to_pinger.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void PingChecker::set_required_passing_pings(size_t n)
{
    required_passing_pings_ = std::max<size_t>(n, 1);
}

// how many pings so far have met the direction change threshold
size_t PingChecker::passing_ping_count() const
{
    return passing_ping_count_;
}

// true if we passed the pinger, false if not
bool PingChecker::new_ping(mil_msgs::msg::ProcessedPing const& new_ping)
{
    // does this ping's direction differ enough from any recent ping?
    bool meets_threshold = false;
    for (auto const& old_ping : recent_pings_)
    {
        if (compare_two_pings(old_ping, new_ping))
        {
            meets_threshold = true;
            break;
        }
    }

    if (meets_threshold)
    {
        passing_ping_count_++;
    }

    // insert new ping into queue
    this->insert_new_ping(new_ping);

    // trim length
    trim_to_n_elements(10);

    // passed the pinger once enough pings met the threshold
    return passing_ping_count_ >= required_passing_pings_;
}

// delete all recent pings
void PingChecker::reset()
{
    recent_pings_.clear();
    passing_ping_count_ = 0;
}

void PingChecker::insert_new_ping(mil_msgs::msg::ProcessedPing const& new_ping)
{
    recent_pings_.push_back(new_ping);
}

void PingChecker::trim_to_n_elements(size_t n)
{
    for (; recent_pings_.size() > n; recent_pings_.pop_front())
    {
    }
}

// true if we passed the pinger, false if not
bool PingChecker::compare_two_pings(mil_msgs::msg::ProcessedPing const& p1, mil_msgs::msg::ProcessedPing const& p2,
                                    int max_degrees)
{
    auto x1 = p1.origin_direction_body.x;
    auto y1 = p1.origin_direction_body.y;
    auto x2 = p2.origin_direction_body.x;
    auto y2 = p2.origin_direction_body.y;

    // normalize them
    auto mag1 = std::sqrt(x1 * x1 + y1 * y1);
    auto mag2 = std::sqrt(x2 * x2 + y2 * y2);
    if (mag1 <= 1e-6 || mag2 <= 1e-6)
    {
        return false;
    }

    x1 /= mag1;
    y1 /= mag1;
    x2 /= mag2;
    y2 /= mag2;

    // compute dot product
    auto dot12 = x1 * x2 + y1 * y2;
    dot12 = std::clamp(dot12, -1.0, 1.0);

    // get angle (it's acos of dot)
    auto angle_rad = std::acos(dot12);
    auto angle_degrees =
        (angle_rad * 180) / 3.14159265358979323846;  // would be better to use a named var not magic numbers idc

    return angle_degrees > max_degrees;
}

SonarFollower::SonarFollower(std::string const& name, const BT::NodeConfig& config) : BT::DecoratorNode(name, config)
{
    get_port_data();

    // sub to ping publisher
    sub_ = ctx_->node->create_subscription<mil_msgs::msg::ProcessedPing>(
        "hydrophones/solved", 10, [this](mil_msgs::msg::ProcessedPing const& msg) { this->topic_cb(msg); });
}

BT::PortsList SonarFollower::providedPorts()
{
    return {
        BT::InputPort<std::shared_ptr<Context>>("ctx"),
        BT::InputPort<bool>("stop_on_first_ping", false, "stop on first ping"),
        BT::InputPort<int>("required_passing_pings", 1,
                           "number of pings that must meet the direction change threshold before success"),
        BT::InputPort<uint32_t>("target_freq", 0, "only follow ping with this HZ; 0 for all pings"),
        BT::InputPort<uint32_t>("target_freq_tol", 0, "accept pings within +/- this many HZ of target_freq"),

        BT::OutputPort<double>("sonar_x"),
        BT::OutputPort<double>("sonar_y"),
        BT::OutputPort<double>("sonar_yaw_deg"),
    };
}

void SonarFollower::get_port_data()
{
    auto ctx_res = getInput<std::shared_ptr<Context>>("ctx");
    if (!ctx_res)
    {
        throw BT::RuntimeError("TopicTicker requires [ctx] input");
    }
    ctx_ = ctx_res.value();

    auto stop_res = getInput<bool>("stop_on_first_ping");
    if (stop_res)
    {
        stop_on_first_ping_ = stop_res.value();
    }

    auto required_res = getInput<int>("required_passing_pings");
    if (required_res)
    {
        required_passing_pings_ = std::max(required_res.value(), 1);
    }
    pc.set_required_passing_pings(static_cast<size_t>(required_passing_pings_));

    auto freq_res = getInput<uint32_t>("target_freq");
    if (freq_res)
    {
        target_freq_ = freq_res.value();
    }

    auto freq_tol_res = getInput<uint32_t>("target_freq_tol");
    if (freq_tol_res)
    {
        target_freq_tol_ = freq_tol_res.value();
    }
}

// each ping we will move and check if done
void SonarFollower::topic_cb(mil_msgs::msg::ProcessedPing const& msg)
{
    if (current_status_ != BT::NodeStatus::RUNNING)
    {
        return;
    }

    if (target_freq_ != 0)
    {
        uint32_t freq_diff =
            (msg.frequency > target_freq_) ? msg.frequency - target_freq_ : target_freq_ - msg.frequency;
        if (freq_diff > target_freq_tol_)
        {
            return;
        }
    }

    if (stop_on_first_ping_)
    {
        current_status_ = BT::NodeStatus::SUCCESS;
        pc.reset();
        return;
    }

    // check ping
    size_t const count_before = pc.passing_ping_count();
    bool passed_the_pinger = pc.new_ping(msg);
    if (pc.passing_ping_count() > count_before)
    {
        RCLCPP_INFO(ctx_->node->get_logger(), "SonarFollower: ping met direction change threshold (%zu/%d)",
                    pc.passing_ping_count(), required_passing_pings_);
    }
    if (passed_the_pinger)
    {
        current_status_ = BT::NodeStatus::SUCCESS;
        pc.reset();
        return;
    }

    // move towards ping
    move_towards_ping(msg);
}

void SonarFollower::move_towards_ping(mil_msgs::msg::ProcessedPing const& new_ping)
{
    // this isn't dry :(
    auto x = new_ping.origin_direction_body.x;
    auto y = new_ping.origin_direction_body.y;
    auto mag = std::sqrt(x * x + y * y);
    if (mag <= 1e-6)
    {
        return;
    }
    x /= mag;
    y /= mag;

    // signed heading to the pinger relative to our current body frame
    auto const yaw_deg = std::atan2(y, x) * 180.0 / M_PI;

    // set outputs
    setOutput("sonar_x", x * 0.5);
    setOutput("sonar_y", y * 0.5);
    setOutput("sonar_yaw_deg", yaw_deg);

    // restart the child so it targets the fresh bearing instead of
    // finishing a move aimed at a stale one
    if (child_node_->status() != BT::NodeStatus::IDLE)
    {
        haltChild();
    }

    auto const child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::FAILURE)
    {
        current_status_ = BT::NodeStatus::FAILURE;
        pc.reset();
    }
}

BT::NodeStatus SonarFollower::tick()
{
    if (current_status_ == BT::NodeStatus::IDLE)
    {
        current_status_ = BT::NodeStatus::RUNNING;
    }

    auto temp = current_status_;
    if (current_status_ == BT::NodeStatus::SUCCESS || current_status_ == BT::NodeStatus::FAILURE)
    {
        current_status_ = BT::NodeStatus::IDLE;
    }

    return temp;
}

// TODO halt
// custom halt needed since node has internal state
void SonarFollower::halt()
{
    BT::DecoratorNode::halt();
    current_status_ = BT::NodeStatus::IDLE;
    pc.reset();
}
