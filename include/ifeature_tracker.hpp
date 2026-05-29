#pragma once

#include "frame.hpp"

class IFeatureTracker
{
public:
    virtual ~IFeatureTracker() = default;

    virtual bool extract(Frame::Ptr prev, Frame::Ptr curr) = 0;

    virtual bool match(
        Frame::Ptr prev,
        Frame::Ptr curr,
        std::vector<cv::DMatch>& good_matches,
        std::vector<cv::Point2f>& pts1,
        std::vector<cv::Point2f>& pts2
    ) = 0;
};
