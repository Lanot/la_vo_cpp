#include "feature_tracker.hpp"

FeatureTracker::FeatureTracker(const TrackerConfig& config)
    : config_(config)
{
    orb_ = cv::ORB::create(
        config_.orb_max_extract_features,          // nfeatures
        1.2f,          // scaleFactor
        8,             // nlevels
        31,            // edgeThreshold
        0,             // firstLevel
        2,             // WTA_K
        cv::ORB::HARRIS_SCORE,
        31,            // patchSize
        20             // fastThreshold
    );

    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);
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
    std::vector<cv::DMatch> matches;

    matcher_->match(
        prev->descriptors,
        curr->descriptors,
        matches
    );

    sortDMatches(matches);

    matches.resize(config_.orb_max_sorted_features);

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

    return good_matches.size() >= config_.orb_min_valid_features;
}
