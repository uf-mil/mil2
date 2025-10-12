#include "missions.hpp"

std::string PassPoleMission::buildTreeXml(MissionParams const &)
{
    return R"BT(
<root BTCPP_format="4" main_tree_to_execute="PassPoleMission">
  <BehaviorTree ID="PassPoleMission">
    <Sequence>
      <!-- 1) Wait for red_pole -->
      <Action ID="WaitForTarget" label="red_pole" min_conf="0.40" timeout_msec="30000" ctx="{ctx}"/>

      <!-- 2) Yaw left until the pole sits +30° to the right of center -->
      <Action ID="AlignBearing"
              label="red_pole"
              desired_bearing_deg="30.0"
              tolerance_deg="4.0"
              fov_deg="90.0"
              max_step_deg="12.0"
              min_conf="0.40"
              ctx="{ctx}"/>

      <!-- 3) Move forward until it's out of view -->
      <Action ID="AdvanceUntilLost"
              label="red_pole"
              step_m="0.35"
              interval_msec="800"
              lose_count="5"
              min_conf="0.35"
              max_steps="50"
              ctx="{ctx}"/>
    </Sequence>
  </BehaviorTree>
</root>
  )BT";
}
