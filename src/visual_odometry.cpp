#include <vector>

#include "visual_odometry.hpp"
#include "visual_odometry_result.hpp"

VisualOdometry::VisualOdometry(const Config& config, const FeatureTrackerORB& tracker, const PoseEstimator& estimator)
    : config_(config), tracker_(tracker), estimator_(estimator)
{
    global_pose_ = Sophus::SE3d();
}

VisualOdometryResult VisualOdometry::process(
    const cv::Mat& image,
    double timestamp)
{
    VisualOdometryResult res;

    res.curr_frame = Frame::create();
    res.curr_frame->image = image.clone();
    res.curr_frame->timestamp = timestamp;

    tracker_.extract(res.curr_frame);

    res.prev_frame = prev_frame_;

    if (!prev_frame_)
    {
        prev_frame_ = res.curr_frame;
        return res;
    }

    res.matched = tracker_.match(
        prev_frame_,
        res.curr_frame,
        res.matches,
        res.pts1,
        res.pts2
    );

    if (!res.matched)
    {
        prev_frame_ = res.curr_frame;
        return res;
    }

    std::vector<uchar> status;
    Sophus::SE3d relative_pose;

    res.estimated = estimator_.estimate(
        res.pts1,
        res.pts2,
        config_.K(),
        relative_pose,
        status
    );

    if (!res.estimated)
    {
        prev_frame_ = res.curr_frame;
        return res;
    }

    global_pose_ = global_pose_ * relative_pose;
    res.curr_frame->pose = global_pose_;

    prev_frame_ = res.curr_frame;
    return res;
}

Sophus::SE3d VisualOdometry::currentPose() const
{
    return global_pose_;
}
