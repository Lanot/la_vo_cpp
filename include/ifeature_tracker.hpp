#pragma once

#include "frame.hpp"

class FeatureTracker
{
public:
    virtual ~FeatureTracker() = default;

    virtual bool extract(Frame::Ptr frame) = 0;

    virtual bool match(
        Frame::Ptr prev,
        Frame::Ptr curr,
        std::vector<cv::DMatch>& good_matches,
        std::vector<cv::Point2f>& pts1,
        std::vector<cv::Point2f>& pts2
    ) = 0;
};
