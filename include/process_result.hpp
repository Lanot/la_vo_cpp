#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "frame.hpp"

struct ProcessResult
{
    bool matched = false;
    bool estimated = false;

    Frame::Ptr curr_frame;
    Frame::Ptr prev_frame;

    Sophus::SE3d relative_pose;
    Sophus::SE3d global_pose;

    std::vector<cv::DMatch> matches;

    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
};
