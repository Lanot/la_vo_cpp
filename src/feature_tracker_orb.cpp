#include "feature_tracker_orb.hpp"

FeatureTrackerORB::FeatureTrackerORB(const TrackerConfig& config)
    : config_(config)
{
    orb_ = cv::ORB::create(
        config_.orb_max_extract_features, // nfeatures
        1.2f, // scaleFactor
        8, // nlevels
        31, // edgeThreshold
        0, // firstLevel
        2, // WTA_K
        cv::ORB::HARRIS_SCORE,
        31, // patchSize
        20 // fastThreshold
    );

    auto cfm = config_.orb_feature_matcher;
    if (cfm == FeatureMatcher::LKOF)
    {
        lkofCriteria_ = cv::makePtr<cv::TermCriteria>(
            cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
            config_.orb_lkof_criteria_max_it,
            config_.orb_lkof_criteria_eps
        );
    }
    else if (cfm == FeatureMatcher::BF || cfm == FeatureMatcher::BF_KNN)
    {
        bfMatcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);
    }
    else if (cfm == FeatureMatcher::FLANN || cfm == FeatureMatcher::FLANN_KNN)
    {
        // ORB requires LSH index: ORB + FLANN-LSH, ORB Descriptor is CV_8U
         flannMatcher_ = cv::makePtr<cv::FlannBasedMatcher>(
            cv::makePtr<cv::flann::LshIndexParams>(
                12, // table_number
                20, // key_size
                2   // multi_probe_level
            )
         );
    }
}

bool FeatureTrackerORB::extract(Frame::Ptr prev, Frame::Ptr curr)
{
    orb_->detectAndCompute(curr->image, cv::noArray(), curr->keypoints, curr->descriptors);

    return !curr->keypoints.empty();
}

bool FeatureTrackerORB::match(
    Frame::Ptr prev,
    Frame::Ptr curr,
    std::vector<cv::Point2f>& pts1,
    std::vector<cv::Point2f>& pts2
)
{
    auto cfm = config_.orb_feature_matcher;
    if (cfm == FeatureMatcher::LKOF)
    {
        estimateOpticalFlowPyrLKMatchesAndFillResults(*lkofCriteria_, prev, curr, pts1, pts2);
    }
    else if (cfm == FeatureMatcher::BF_KNN || cfm == FeatureMatcher::FLANN_KNN)
    {
        std::vector<std::vector<cv::DMatch>> knnMatches;

        if (cfm == FeatureMatcher::BF_KNN)
        {
            bfMatcher_->knnMatch(prev->descriptors, curr->descriptors, knnMatches, 2);
        }
        else
        {
            flannMatcher_->knnMatch(prev->descriptors, curr->descriptors, knnMatches, 2);
        }

        filterKnnMatchesAndFillResults(config_.orb_knn_dist_k, prev, curr, knnMatches, pts1, pts2);
    }
    else if (cfm == FeatureMatcher::BF || cfm == FeatureMatcher::FLANN)
    {
        std::vector<cv::DMatch> matches;

        if (cfm == FeatureMatcher::BF)
        {
            bfMatcher_->match(prev->descriptors, curr->descriptors, matches);
        }
        else
        {
            flannMatcher_->match(prev->descriptors, curr->descriptors, matches);
        }

        sortMatches(matches);
        matches.resize(config_.orb_max_sorted_simple_features);

        filterMatchesAndFillResults(config_.orb_max_dist_simple, prev, curr, matches, pts1, pts2);
    }

    return pts2.size() >= config_.orb_min_valid_features;
}
