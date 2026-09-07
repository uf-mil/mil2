#pragma once

#include <optional>
#include <string>

#include "detection_gate.hpp"

#include <yolo_msgs/msg/detection_array.hpp>

// Pixel area (bbox.size.x * bbox.size.y) of the highest-confidence detection whose
// class_name == label and score >= min_conf, or nullopt if none match. Used by the
// grasp-by-scale check: a held object keeps its apparent size after the lift, while a
// missed one (left on the table) shrinks as the sub rises away from it.
inline std::optional<double> best_bbox_area(yolo_msgs::msg::DetectionArray const& arr, std::string const& label,
                                            double min_conf)
{
    auto const* d = detection_gate::best_detection(arr, label, min_conf);
    if (!d)
    {
        return std::nullopt;
    }
    return static_cast<double>(d->bbox.size.x) * static_cast<double>(d->bbox.size.y);
}
