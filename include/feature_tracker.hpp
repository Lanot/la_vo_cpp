#pragma once

#include "frame.hpp"

class FeatureTracker
{
public:
    FeatureTracker();

    bool extract(Frame::Ptr frame);

    bool match(
        Frame::Ptr prev,
        Frame::Ptr curr,
        std::vector<cv::DMatch>& good_matches,
        std::vector<cv::Point2f>& pts1,
        std::vector<cv::Point2f>& pts2
    );

private:
    cv::Ptr<cv::ORB> orb_;
};
