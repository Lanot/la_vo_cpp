#pragma once

#include "config.hpp"
#include "frame.hpp"
#include "utils.hpp"
#include "ifeature_tracker.hpp"

class FeatureTracker : public IFeatureTracker
{
public:
    FeatureTracker(const TrackerConfig& config);

    bool extract(Frame::Ptr frame);

    bool match(
        Frame::Ptr prev,
        Frame::Ptr curr,
        std::vector<cv::DMatch>& good_matches,
        std::vector<cv::Point2f>& pts1,
        std::vector<cv::Point2f>& pts2
    );

protected:
    TrackerConfig config_;

    cv::Ptr<cv::BFMatcher> matcher_;
    cv::Ptr<cv::ORB> orb_;
};
