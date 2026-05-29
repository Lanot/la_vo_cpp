#pragma once

#include "config.hpp"
#include "frame.hpp"
#include "utils.hpp"
#include "ifeature_tracker.hpp"

class FeatureTrackerSuperPoint : public IFeatureTracker
{
public:
    FeatureTrackerSuperPoint(const TrackerConfig& config);

    bool extract(Frame::Ptr prev, Frame::Ptr curr);

    bool match(
        Frame::Ptr prev,
        Frame::Ptr curr,
        std::vector<cv::Point2f>& pts1,
        std::vector<cv::Point2f>& pts2
    );

protected:
    TrackerConfig config_;
    // cv::Ptr<cv::SIFT> super_point_;
    cv::Ptr<cv::BFMatcher> bfMatcher_;
    cv::Ptr<cv::FlannBasedMatcher> flannMatcher_;
};
