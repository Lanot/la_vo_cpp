#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "frame.hpp"

struct VisualOdometryResult
{
    bool matched = false;
    bool estimated = false;

    Frame::Ptr curr_frame;
    Frame::Ptr prev_frame;

    std::vector<cv::DMatch> matches;

    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
};
