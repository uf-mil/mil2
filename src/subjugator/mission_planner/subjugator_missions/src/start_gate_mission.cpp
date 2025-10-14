#include "missions.hpp"

std::string StartGateMission::buildTreeXml(MissionParams const &)
{
    return R"BT(
<root BTCPP_format="4" main_tree_to_execute="StartGateMission">
  <BehaviorTree ID="StartGateMission">
    <Sequence>

      <!-- 1) Survey sweep 60 degrees each way-->
      <Parallel success_count="1" failure_count="2">
        <RetryUntilSuccessful num_attempts="-1">
          <Sequence>
            <Delay delay_msec="200"><AlwaysSuccess/></Delay>
            <Condition ID="DetectTarget"
                      label="shark"
                      min_conf="0.40"
                      ctx="{ctx}"/>
          </Sequence>
        </RetryUntilSuccessful>

        <Repeat num_cycles="-1">
        <Sequence>

          <!-- +15 degrees to 60 degrees -->
          <Repeat num_cycles="4">
            <Fallback>
              <SubTree ID="RelativeMove" _autoremap="true"
                      x="0.0" y="0.0" z="0.0"
                      qx="0.0" qy="0.0"
                      qz="0.1305261922" qw="0.9914448614"
                      pos_tol="0.25" ori_tol_deg="6.0" yaw_only="true"
                      msec="4000"
                      ctx="{ctx}"/>
              <AlwaysSuccess/>
            </Fallback>
          </Repeat>

          <!-- -15 degrees to -60 degrees -->
          <Repeat num_cycles="8">
            <Fallback>
              <SubTree ID="RelativeMove" _autoremap="true"
                      x="0.0" y="0.0" z="0.0"
                      qx="0.0" qy="0.0"
                      qz="-0.1305261922" qw="0.9914448614"
                      pos_tol="0.25" ori_tol_deg="6.0" yaw_only="true"
                      msec="4000"
                      ctx="{ctx}"/>
              <AlwaysSuccess/>
            </Fallback>
          </Repeat>

          <!-- +15° x4 (back to center) -->
          <Repeat num_cycles="4">
            <Fallback>
              <SubTree ID="RelativeMove" _autoremap="true"
                      x="0.0" y="0.0" z="0.0"
                      qx="0.0" qy="0.0"
                      qz="0.1305261922" qw="0.9914448614"
                      pos_tol="0.25" ori_tol_deg="6.0" yaw_only="true"
                      msec="4000"
                      ctx="{ctx}"/>
              <AlwaysSuccess/>
            </Fallback>
          </Repeat>

        </Sequence>
        </Repeat>

      </Parallel>

      <!-- 2) Hone orientation toward the shark with offset -->
      <Action ID="HoneBearing"
              label="shark"
              offset_deg="0.0"
              tolerance_deg="3.0"
              fov_deg="90.0"
              max_step_deg="10.0"
              min_conf="0.40"
              ctx="{ctx}"/>

      <!-- 3) Then down and forward (keep from your previous mission) -->
      <SubTree ID="RelativeMove" _autoremap="true"
               x="0.0" y="0.0" z="-0.5"
               qx="0.0" qy="0.0" qz="0.0" qw="1.0"
               pos_tol="0.25" ori_tol_deg="12.0" msec="10000"
               ctx="{ctx}"/>
      <SubTree ID="RelativeMove" _autoremap="true"
               x="3.0" y="0.0" z="0.0"
               qx="0.0" qy="0.0" qz="0.0" qw="1.0"
               pos_tol="0.25" ori_tol_deg="180.0" msec="45000"
               ctx="{ctx}"/>

      <!-- 4) Short second survey (±45°), no detection gating -->
      <Sequence>
        <SubTree ID="RelativeMove" _autoremap="true"
                 x="0.0" y="0.0" z="0.0"
                 qx="0.0" qy="0.0" qz="0.3826834324" qw="0.9238795325"
                 pos_tol="0.25" ori_tol_deg="8.0" msec="20000"
                 ctx="{ctx}"/>
        <SubTree ID="RelativeMove" _autoremap="true"
                 x="0.0" y="0.0" z="0.0"
                 qx="0.0" qy="0.0" qz="-0.3826834324" qw="0.9238795325"
                 pos_tol="0.25" ori_tol_deg="8.0" msec="20000"
                 ctx="{ctx}"/>
        <SubTree ID="RelativeMove" _autoremap="true"
                 x="0.0" y="0.0" z="0.0"
                 qx="0.0" qy="0.0" qz="-0.3826834324" qw="0.9238795325"
                 pos_tol="0.25" ori_tol_deg="8.0" msec="20000"
                 ctx="{ctx}"/>
        <SubTree ID="RelativeMove" _autoremap="true"
                 x="0.0" y="0.0" z="0.0"
                 qx="0.0" qy="0.0" qz="0.3826834324" qw="0.9238795325"
                 pos_tol="0.25" ori_tol_deg="8.0" msec="20000"
                 ctx="{ctx}"/>
      </Sequence>

    </Sequence>
  </BehaviorTree>
</root>
)BT";
}
