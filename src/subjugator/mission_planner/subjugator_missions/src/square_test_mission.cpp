#include "missions.hpp"

// Main tree only; it calls the previously-registered subtree "RelativeMove".
std::string SquareTestMission::buildTreeXml(MissionParams const &)
{
    return R"BT(
        <root main_tree_to_execute="SquareTestMission">
        <BehaviorTree ID="SquareTestMission">
        <Sequence>

            <SubTree ID="RelativeMove"
                    rx="0.0" ry="0.0" rz="-0.2"
                    rqx="0.0" rqy="0.0" rqz="0.0" rqw="1.0"
                    pos_tol="0.25" ori_tol_deg="12.0" msec="5000"/>

            <SubTree ID="RelativeMove"
                    rx="1.5" ry="0.0" rz="0.0"
                    rqx="0.0" rqy="0.0" rqz="0.0" rqw="1.0"
                    pos_tol="0.25" ori_tol_deg="12.0" msec="5000"/>

            <SubTree ID="RelativeMove"
                    rx="0.0" ry="1.5" rz="0.0"
                    rqx="0.0" rqy="0.0" rqz="0.0" rqw="1.0"
                    pos_tol="0.25" ori_tol_deg="12.0" msec="5000"/>

            <SubTree ID="RelativeMove"
                    rx="-1.5" ry="0.0" rz="0.0"
                    rqx="0.0" rqy="0.0" rqz="0.0" rqw="1.0"
                    pos_tol="0.25" ori_tol_deg="12.0" msec="5000"/>

            <SubTree ID="RelativeMove"
                    rx="0.0" ry="-1.5" rz="0.0"
                    rqx="0.0" rqy="0.0" rqz="0.0" rqw="1.0"
                    pos_tol="0.25" ori_tol_deg="12.0" msec="5000"/>

        </Sequence>
        </BehaviorTree>
        </root>
    )BT";
}
