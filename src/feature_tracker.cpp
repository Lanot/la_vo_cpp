#include "feature_tracker.hpp"

FeatureTracker::FeatureTracker()
{
    orb_ = cv::ORB::create(2000);
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

    double min_dist = 9999;

    for (auto& m : matches)
    {
        if (m.distance < min_dist)
            min_dist = m.distance;
    }

    for (auto& m : matches)
    {
        if (m.distance <= std::max(2.0 * min_dist, 30.0))
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

    return good_matches.size() >= 20;
}
