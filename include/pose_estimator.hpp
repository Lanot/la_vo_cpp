#pragma once

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include "config.hpp"

class PoseEstimator
{
protected:
    PoseEstimatorConfig config_;

public:
    explicit PoseEstimator(const PoseEstimatorConfig& config);

    bool estimate(
        const std::vector<cv::Point2f>& pts1,
        const std::vector<cv::Point2f>& pts2,
        const cv::Mat& K,
        Sophus::SE3d& relative_pose
    );
};
