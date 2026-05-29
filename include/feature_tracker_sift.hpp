#pragma once

#include "config.hpp"
#include "frame.hpp"
#include "utils.hpp"
#include "ifeature_tracker.hpp"

class FeatureTrackerSIFT : public IFeatureTracker
{
public:
    FeatureTrackerSIFT(const TrackerConfig& config);

    bool extract(Frame::Ptr prev, Frame::Ptr curr);

    bool match(
        Frame::Ptr prev,
        Frame::Ptr curr,
        std::vector<cv::DMatch>& resMatches,
        std::vector<cv::Point2f>& resPrevPts,
        std::vector<cv::Point2f>& resCurrPts
    );

protected:
    TrackerConfig config_;
    cv::Ptr<cv::SIFT> sift_;

    cv::Ptr<cv::TermCriteria> lkofCriteria_;
    cv::Ptr<cv::BFMatcher> bfMatcher_;
    cv::Ptr<cv::FlannBasedMatcher> flannMatcher_;
};
