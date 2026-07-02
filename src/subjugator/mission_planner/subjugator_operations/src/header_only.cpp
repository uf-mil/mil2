#include "context.hpp"

#include "count_when_ticked.hpp"
#include "go_to_pinger.hpp"
#include "pitch_style.hpp"
#include "roll_style.hpp"
#include "topic_ticker.hpp"
#include "yaw_style.hpp"

REGISTER(CountWhenTicked)
REGISTER(SonarFollower)
REGISTER(PitchStyle)
REGISTER(RollStyle)
using TopicTickerOdom = TopicTicker<nav_msgs::msg::Odometry>;
REGISTER(TopicTickerOdom)
REGISTER(YawStyle)
