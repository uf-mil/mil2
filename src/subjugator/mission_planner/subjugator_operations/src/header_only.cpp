#include "context.hpp"
#include "count_when_ticked.hpp"
#include "go_to_pinger.hpp"
#include "lookup_waypoint.hpp"
#include "remember_waypoint.hpp"
#include "spin_style.hpp"  // RollStyle + PitchStyle
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
