#include "feature_tracker_sift.hpp"

FeatureTrackerSIFT::FeatureTrackerSIFT(const TrackerConfig& config)
    : config_(config)
{
    sift_ = cv::SIFT::create(
        config_.sift_max_extract_features, // nfeatures
        3, // nOctaveLayers
        0.04, // contrastThreshold
        10, // edgeThreshold
        1.6 // sigma
    );

    auto cfm = config_.sift_feature_matcher;
    if (cfm == FeatureMatcher::LKOF)
    {
        lkofCriteria_ = cv::makePtr<cv::TermCriteria>(
            cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
            config_.sift_lkof_criteria_max_it,
            config_.sift_lkof_criteria_eps
        );
    }
    else if (cfm == FeatureMatcher::BF || cfm == FeatureMatcher::BF_KNN)
    {
        bfMatcher_ = cv::BFMatcher::create(cv::NORM_L2, false);
    }
    else if (cfm == FeatureMatcher::FLANN || cfm == FeatureMatcher::FLANN_KNN)
    {
        // FLANN parameters for SIFT, SIFT Descriptor is CV_32F
        flannMatcher_ = cv::makePtr<cv::FlannBasedMatcher>(
            cv::makePtr<cv::flann::KDTreeIndexParams>(5),
            cv::makePtr<cv::flann::SearchParams>(50)
        );
    }
}

bool FeatureTrackerSIFT::extract(Frame::Ptr prev, Frame::Ptr curr)
{
    sift_->detectAndCompute(curr->image, cv::noArray(), curr->keypoints, curr->descriptors);

    return !curr->keypoints.empty();
}

bool FeatureTrackerSIFT::match(
    Frame::Ptr prev,
    Frame::Ptr curr,
    std::vector<cv::DMatch>& resMatches,
    std::vector<cv::Point2f>& resPrevPts,
    std::vector<cv::Point2f>& resCurrPts
)
{
    auto cfm = config_.sift_feature_matcher;
    if (cfm == FeatureMatcher::LKOF)
    {
        estimateOpticalFlowPyrLKMatchesAndFillResults(*lkofCriteria_, prev, curr, resPrevPts, resCurrPts);
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

        filterKnnMatchesAndFillResults(config_.sift_knn_dist_k, prev, curr, knnMatches, resMatches, resPrevPts,
                                       resCurrPts);
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
        matches.resize(config_.sift_max_sorted_simple_features);

        filterSimpleMatchesAndFillResults(config_.sift_max_dist_simple, prev, curr, matches, resMatches, resPrevPts,
                                          resCurrPts);
    }

    return resCurrPts.size() >= config_.sift_min_valid_features;
}
