#include <vector>
#include <iostream>

#include "visual_odometry.hpp"
#include "process_result.hpp"

VisualOdometry::VisualOdometry(const CameraIntrinsics& camera_intrinsics)
    : camera_intrinsics_(camera_intrinsics)
{
    global_pose_ = Sophus::SE3d();
}

ProcessResult VisualOdometry::process(
    const cv::Mat& image,
    double timestamp)
{
    ProcessResult res;

    current_ = Frame::create();
    current_->image = image.clone();
    current_->timestamp = timestamp;

    tracker_.extract(current_);

    res.prev_frame = previous_;
    res.curr_frame = current_;

    if (!previous_)
    {
        previous_ = current_;
        return res;
    }

    res.matched = tracker_.match(
        previous_,
        current_,
        res.matches,
        res.pts1,
        res.pts2
    );

    if (!res.matched)
    {
        previous_ = current_;
        return res;
    }

    std::vector<uchar> status;

    res.estimated = estimator_.estimate(
        res.pts1,
        res.pts2,
        camera_intrinsics_.K(),
        res.relative_pose,
        status
    );

    if (!res.estimated)
    {
        previous_ = current_;
        return res;
    }

    global_pose_ = global_pose_ * res.relative_pose;
    current_->pose = global_pose_;

    res.global_pose = global_pose_;
    res.prev_frame = previous_;
    res.curr_frame = current_;

    previous_ = current_;
    return res;
}

Sophus::SE3d VisualOdometry::currentPose() const
{
    return global_pose_;
}
