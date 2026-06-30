#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

#include "context.hpp"
#include "operations.hpp"

// DetectOrangeMarker
//
// A simple BT::ConditionNode (ticks once, no waiting) that checks the
// front camera for the orange path marker using HSV color thresholding,
// the same approach as path_marker_heading.py uses on the down camera.
//
// Returns SUCCESS if an orange blob big enough to plausibly be the marker
// is visible in the current front cam frame, FAILURE otherwise.
//
// Used two ways in the mission XML:
//   1. Inside a RetryUntilSuccessful, to spin/search until found
//   2. Inside a RetryUntilFailure, to detect when the marker disappears
//      (meaning it's now under the sub)
//
// XML usage:
//   <Condition ID="DetectOrangeMarker" ctx="{ctx}" min_area_px="1000.0"/>

class DetectOrangeMarker : public BT::ConditionNode, public OperationBase
{
  public:
    DetectOrangeMarker(std::string const& name, BT::NodeConfiguration const& cfg)
      : BT::ConditionNode(name, cfg), OperationBase(cfg)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<Context>>("ctx"),
            // TODO: placeholder HSV range, same as path_marker_heading.py --
            // replace once Dean's test data packet gives us real values
            BT::InputPort<double>("min_area_px", 1000.0, "Minimum contour area in pixels to count as a detection"),
        };
    }

    BT::NodeStatus tick() override;
};
