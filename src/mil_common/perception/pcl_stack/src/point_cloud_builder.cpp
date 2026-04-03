#include "pcl_stack/point_cloud_builder.hpp"

namespace pcodar
{
PointCloudCircularBuffer::PointCloudCircularBuffer() : mega_cloud_(std::make_shared<point_cloud>())
{
}

void PointCloudCircularBuffer::add_point_cloud(point_cloud_ptr const& pc)
{
    // Add new cloud to buffer
    prev_clouds_.push_back(pc);

    // Don't construct mega cloud until buffer of recent clouds is full
    if (!prev_clouds_.full())
        return;

    //  Assemble cloud as union of buffered clouds
    mega_cloud_->clear();
    for (auto const& cloud : prev_clouds_)
    {
        *mega_cloud_ += *cloud;
    }
}

void PointCloudCircularBuffer::clear()
{
    mega_cloud_->clear();
    prev_clouds_.clear();
}

void PointCloudCircularBuffer::update_config(Config const& config)
{
    prev_clouds_.set_capacity(config.accumulator_number_persistant_clouds);
}

point_cloud_ptr PointCloudCircularBuffer::get_point_cloud()
{
    // Only valid once the buffer has filled to capacity at least once
    if (!prev_clouds_.full())
        return nullptr;
    return mega_cloud_;
}

}  // namespace pcodar
