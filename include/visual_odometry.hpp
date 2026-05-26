#pragma once

#include "camera_intrinsics.hpp"
#include "feature_tracker.hpp"
#include "frame.hpp"
#include "pose_estimator.hpp"
#include "process_result.hpp"

class VisualOdometry
{
public:
    explicit VisualOdometry(const CameraIntrinsics& camera_intrinsics);

    ProcessResult process(const cv::Mat& image, double timestamp);

    Sophus::SE3d currentPose() const;

private:
    CameraIntrinsics camera_intrinsics_;

    Frame::Ptr previous_;
    Frame::Ptr current_;

    FeatureTracker tracker_;
    PoseEstimator estimator_;

    Sophus::SE3d global_pose_;

    double scale_ = 1.0;
};
