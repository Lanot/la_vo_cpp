#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

class Frame
{
public:
    using Ptr = std::shared_ptr<Frame>;

    uint64_t id = 0;
    double timestamp = 0.0;

    cv::Mat image;

    std::vector<cv::KeyPoint> keypoints; // extracted
    cv::Mat descriptors; // extracted
    std::vector<cv::Point2f> pts; // matched points

    Sophus::SE3d pose = Sophus::SE3d();

    static Ptr create();

protected:
    static uint64_t next_id_;
};
