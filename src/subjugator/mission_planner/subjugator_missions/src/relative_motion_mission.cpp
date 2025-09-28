#include "missions.hpp"

// Returns a single <BehaviorTree> that defines the reusable subtree.
// IMPORTANT: no <root> wrapper here. We'll register this with the factory first.
std::string RelativeMotionMission::buildTreeXml(MissionParams const &)
{
    // Ports (all strings are BT ports):
    // Inputs to subtree: rx,ry,rz, rqx,rqy,rqz,rqw, pos_tol, ori_tol_deg, msec
    // Internal wiring: PublishGoalPose exposes abs_* which feed AtGoalPose
    return R"BT(
    <BehaviorTree ID="RelativeMove">
      <Sequence>
        <Action ID="PublishGoalPose"
                x="{rx}" y="{ry}" z="{rz}"
                qx="{rqx}" qy="{rqy}" qz="{rqz}" qw="{rqw}"
                relative="true"
                abs_x="{ax}" abs_y="{ay}" abs_z="{az}"
                abs_qx="{aqx}" abs_qy="{aqy}" abs_qz="{aqz}" abs_qw="{aqw}"/>

        <Timeout msec="{msec}">
          <Condition ID="AtGoalPose"
                    x="{ax}" y="{ay}" z="{az}"
                    qx="{aqx}" qy="{aqy}" qz="{aqz}" qw="{aqw}"
                    pos_tol="{pos_tol}" ori_tol_deg="{ori_tol_deg}"/>
        </Timeout>
      </Sequence>
    </BehaviorTree>
  )BT";
}
