#pragma once

#include "config.hpp"
#include "feature_tracker.hpp"
#include "frame.hpp"
#include "pose_estimator.hpp"
#include "process_result.hpp"

class VisualOdometry
{
public:
    explicit VisualOdometry(const Config& config);

    ProcessResult process(const cv::Mat& image, double timestamp);

    Sophus::SE3d currentPose() const;

protected:
    Config config_;

    Frame::Ptr prev_frame_;
    Sophus::SE3d global_pose_;

    FeatureTracker tracker_;
    PoseEstimator estimator_;

    double scale_ = 1.0;
};
