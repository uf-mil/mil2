#include "missions.hpp"

std::string SquareTestMission::buildTreeXml(MissionParams const &)
{
    return R"BT(
<root BTCPP_format="4" main_tree_to_execute="SquareTestMission">
    <BehaviorTree ID="SquareTestMission">
        <Sequence>
            <SubTree ID="RelativeMove"
                x="0.0" y="0.0" z="-0.2"
                qx="0.0" qy="0.0" qz="0.0" qw="1.0"
                pos_tol="0.25" ori_tol_deg="12.0" msec="5000"
                ctx="{ctx}"/>
            <SubTree ID="RelativeMove"
                x="1.5" y="0.0" z="0.0"
                qx="0.0" qy="0.0" qz="0.0" qw="1.0"
                pos_tol="0.25" ori_tol_deg="12.0" msec="5000"
                ctx="{ctx}"/>
            <SubTree ID="RelativeMove"
                x="0.0" y="1.5" z="0.0"
                qx="0.0" qy="0.0" qz="0.0" qw="1.0"
                pos_tol="0.25" ori_tol_deg="12.0" msec="5000"
                ctx="{ctx}"/>
            <SubTree ID="RelativeMove"
                x="-1.5" y="0.0" z="0.0"
                qx="0.0" qy="0.0" qz="0.0" qw="1.0"
                pos_tol="0.25" ori_tol_deg="12.0" msec="5000"
                ctx="{ctx}"/>
            <SubTree ID="RelativeMove"
                x="0.0" y="-1.5" z="0.0"
                qx="0.0" qy="0.0" qz="0.0" qw="1.0"
                pos_tol="0.25" ori_tol_deg="12.0" msec="5000"
                ctx="{ctx}"/>
        </Sequence>
    </BehaviorTree>
</root>
    )BT";
}
