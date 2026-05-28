#include <vector>

#include "visual_odometry.hpp"
#include "visual_odometry_result.hpp"

VisualOdometry::VisualOdometry(
    std::shared_ptr<Config> config,
    std::unique_ptr<IFeatureTracker> tracker,
    std::unique_ptr<PoseEstimator> estimator
) : config_(config),
    tracker_(std::move(tracker)),
    estimator_(std::move(estimator))
{
    global_pose_ = Sophus::SE3d();
}

VisualOdometryResult VisualOdometry::process(const Frame::Ptr& frame, double scale)
{
    VisualOdometryResult res;

    res.curr_frame = frame;

    tracker_->extract(res.curr_frame);

    res.prev_frame = prev_frame_;

    if (!prev_frame_)
    {
        prev_frame_ = res.curr_frame;
        return res;
    }

    res.matched = tracker_->match(
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

    Sophus::SE3d relative_pose;

    res.estimated = estimator_->estimate(res.pts1, res.pts2, config_->K(),relative_pose, scale);
    if (!res.estimated)
    {
        prev_frame_ = res.curr_frame;
        return res;
    }

    // global_pose : goes opposite side in compare to GT
    //global_pose_ = global_pose_ * relative_pose;

    // fix for mono vo
    global_pose_ = global_pose_ * relative_pose.inverse();

    res.curr_frame->pose = global_pose_;

    prev_frame_ = res.curr_frame;
    return res;
}

Sophus::SE3d VisualOdometry::currentPose() const
{
    return global_pose_;
}
