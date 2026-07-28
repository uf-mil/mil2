#include "context.hpp"
#include "count_when_ticked.hpp"
#include "go_to_pinger.hpp"
#include "lookup_waypoint.hpp"
#include "pitch_style.hpp"
#include "remember_waypoint.hpp"
#include "roll_style.hpp"
#include "topic_ticker.hpp"
#include "yaw_style.hpp"

REGISTER(CountWhenTicked)
REGISTER(SonarFollower)
REGISTER(LookupWaypoint)
REGISTER(PitchStyle)
REGISTER(RememberWaypoint)
REGISTER(RollStyle)
using TopicTickerOdom = TopicTicker<nav_msgs::msg::Odometry>;
REGISTER(TopicTickerOdom)
REGISTER(YawStyle)
