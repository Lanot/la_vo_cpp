#include "feature_tracker.hpp"

FeatureTracker::FeatureTracker(const TrackerConfig& config)
    : config_(config)
{
    orb_ = cv::ORB::create(config_.max_extract_features);
}

bool FeatureTracker::extract(Frame::Ptr frame)
{
    orb_->detectAndCompute(
        frame->image,
        cv::noArray(),
        frame->keypoints,
        frame->descriptors
    );

    return !frame->keypoints.empty();
}

bool FeatureTracker::match(
    Frame::Ptr prev,
    Frame::Ptr curr,
    std::vector<cv::DMatch>& good_matches,
    std::vector<cv::Point2f>& pts1,
    std::vector<cv::Point2f>& pts2)
{
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    std::vector<cv::DMatch> matches;

    matcher.match(
        prev->descriptors,
        curr->descriptors,
        matches
    );

    sortDMatches(matches);

    matches.resize(config_.max_sorted_features);

    for (auto& m : matches)
    {
        if (m.distance <= config_.orb_max_distance)
        {
            good_matches.push_back(m);

            pts1.push_back(
                prev->keypoints[m.queryIdx].pt
            );

            pts2.push_back(
                curr->keypoints[m.trainIdx].pt
            );
        }
    }

    return good_matches.size() >= config_.min_valid_features;
}
