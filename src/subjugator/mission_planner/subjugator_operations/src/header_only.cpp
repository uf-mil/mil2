#include "context.hpp"
#include "count_when_ticked.hpp"
#include "go_to_pinger.hpp"
#include "lawnmower_survey.hpp"
#include "light_follower.hpp"
#include "log_light.hpp"
#include "lookup_waypoint.hpp"
#include "new_light.hpp"
#include "pitch_style.hpp"
#include "remember_waypoint.hpp"
#include "roll_style.hpp"
#include "sort_lights.hpp"
#include "topic_ticker.hpp"
#include "yaw_style.hpp"

REGISTER(CountWhenTicked)
REGISTER(LawnmowerSurvey)
REGISTER(LightFollower)
REGISTER(LogLight)
REGISTER(LookupWaypoint)
REGISTER(NewLight)
REGISTER(PitchStyle)
REGISTER(RememberWaypoint)
REGISTER(RollStyle)
REGISTER(SonarFollower)
REGISTER(SortLights)
using TopicTickerOdom = TopicTicker<nav_msgs::msg::Odometry>;
REGISTER(TopicTickerOdom)
REGISTER(YawStyle)
