#pragma once

#include "config.hpp"
#include "feature_tracker.hpp"
#include "frame.hpp"
#include "pose_estimator.hpp"
#include "visual_odometry_result.hpp"

class VisualOdometry
{
public:
    explicit VisualOdometry(Config& config, FeatureTracker& tracker, PoseEstimator& estimator);

    VisualOdometryResult process(const cv::Mat& image, double timestamp);

    Sophus::SE3d currentPose() const;

protected:
    Frame::Ptr prev_frame_;
    Sophus::SE3d global_pose_;

    Config config_;
    FeatureTracker& tracker_;
    PoseEstimator& estimator_;

    double scale_ = 1.0;
};
