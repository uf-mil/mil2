#include "missions.hpp"

std::string MovementMission::buildTreeXml(MissionParams const &)
{
    // Minimal demo: publish a goal, then wait until AtGoalPose succeeds with retry + timeout.
    // You can bind x,y,z,quaternion via blackboard elsewhere; here we just hardcode a sample.
    return R"BT(
            <root main_tree_to_execute="MovementMission">
            <BehaviorTree ID="MovementMission">
                <Sequence>
                <Action ID="PublishGoalPose"
                        x="5.0" y="2.0" z="0.0"
                        qx="0.0" qy="0.0" qz="0.0" qw="1.0"
                        relative="false"/>

                <Timeout msec="500">
                    <Condition ID="AtGoalPose"
                                x="5.0" y="2.0" z="0.0"
                                qx="0.0" qy="0.0" qz="0.0" qw="1.0"
                                pos_tol="0.25" ori_tol_deg="12.0"/>
                </Timeout>
                </Sequence>
            </BehaviorTree>
            </root>
        )BT";
}
