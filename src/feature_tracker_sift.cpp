#include "feature_tracker_sift.hpp"

FeatureTrackerSIFT::FeatureTrackerSIFT(const TrackerConfig& config)
    : config_(config)
{
    sift_ = cv::SIFT::create(
        config_.sift_max_extract_features,  // nfeatures
        3, // nOctaveLayers
        0.04,  // contrastThreshold
        10, // edgeThreshold
        1.6); // sigma
    

    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);
}

bool FeatureTrackerSIFT::extract(Frame::Ptr frame)
{
    sift_->detectAndCompute(
        frame->image,
        cv::noArray(),
        frame->keypoints,
        frame->descriptors
    );

    return !frame->keypoints.empty();
}

bool FeatureTrackerSIFT::match(
    Frame::Ptr prev,
    Frame::Ptr curr,
    std::vector<cv::DMatch>& good_matches,
    std::vector<cv::Point2f>& pts1,
    std::vector<cv::Point2f>& pts2)
{
    std::vector<std::vector<cv::DMatch>> matches2d;

    matcher_->knnMatch(prev->descriptors, curr->descriptors, matches2d, 2);

    for (std::vector<cv::DMatch>& m : matches2d)
    {
        if (m[0].distance < config_.sift_knn_distance_k * m[1].distance) // && (m[0].distance < config_.sift_max_distance))
        {
            good_matches.push_back(m[0]);

            pts1.push_back(
                prev->keypoints[m[0].queryIdx].pt
            );

            pts2.push_back(
                curr->keypoints[m[0].trainIdx].pt
            );
        }
    }

    return good_matches.size() >= config_.sift_min_valid_features;
}
