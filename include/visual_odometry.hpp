#pragma once

#include "config.hpp"
#include "feature_tracker_orb.hpp"
#include "frame.hpp"
#include "pose_estimator.hpp"
#include "visual_odometry_result.hpp"

class VisualOdometry
{
public:
    explicit VisualOdometry(
        std::shared_ptr<Config> config,
        std::unique_ptr<IFeatureTracker> tracker,
        std::unique_ptr<PoseEstimator> estimator
    );

    VisualOdometryResult process(const Frame::Ptr& frame, double scale = 1.0);

    Sophus::SE3d currentPose() const;

protected:
    Frame::Ptr prev_frame_ = nullptr;
    Sophus::SE3d global_pose_;

    std::shared_ptr<Config> config_;
    std::unique_ptr<IFeatureTracker> tracker_;
    std::unique_ptr<PoseEstimator> estimator_;

    double scale_ = 1.0;
};
