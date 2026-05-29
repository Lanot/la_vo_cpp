#include "feature_tracker_super_point.hpp"

FeatureTrackerSuperPoint::FeatureTrackerSuperPoint(const TrackerConfig& config)
    : config_(config)
{
    // super_point_ = cv::SIFT::create(
    //     config_.super_point_max_extract_features,  // nfeatures
    //     3,     // nOctaveLayers
    //     0.04,  // contrastThreshold
    //     10,    // edgeThreshold
    //     1.6    // sigma
    // );
    //
    // auto cfm = config_.super_point_feature_matcher;
    // if (cfm == FeatureMatcher::BF || cfm == FeatureMatcher::BF_KNN)
    // {
    //     bfMatcher_ = cv::BFMatcher::create(cv::NORM_L2, false);
    // }
    // else
    // {
    //     // FLANN parameters for SIFT, SIFT Descriptor is CV_32F
    //     flannMatcher_ = cv::makePtr<cv::FlannBasedMatcher>(
    //         cv::makePtr<cv::flann::KDTreeIndexParams>(5),
    //         cv::makePtr<cv::flann::SearchParams>(50)
    //     );
    // }
}

bool FeatureTrackerSuperPoint::extract(Frame::Ptr prev, Frame::Ptr curr)
{
    // super_point_->detectAndCompute(frame->image, cv::noArray(), frame->keypoints, frame->descriptors);

    return !curr->keypoints.empty();
}

bool FeatureTrackerSuperPoint::match(
    Frame::Ptr prev,
    Frame::Ptr curr,
    std::vector<cv::DMatch>& resMatches,
    std::vector<cv::Point2f>& resPrevPts,
    std::vector<cv::Point2f>& resCurrPts
)
{
    // auto cfm = config_.super_point_feature_matcher;
    // if (cfm == FeatureMatcher::BF_KNN || cfm == FeatureMatcher::FLANN_KNN)
    // {
    //     std::vector<std::vector<cv::DMatch>> knnMatches;
    //
    //     if (cfm == FeatureMatcher::BF_KNN)
    //     {
    //         bfMatcher_->knnMatch(prev->descriptors, curr->descriptors, knnMatches, 2);
    //     }
    //     else
    //     {
    //         flannMatcher_->knnMatch(prev->descriptors, curr->descriptors, knnMatches, 2);
    //     }
    //
    //     filterKnnMatchesAndFillResults(config_.super_point_knn_dist_k, prev, curr, knnMatches, resMatches, resPrevPts, resCurrPts);
    // }
    // else
    // {
    //     std::vector<cv::DMatch> matches;
    //
    //     if (cfm == FeatureMatcher::BF)
    //     {
    //         bfMatcher_->match(prev->descriptors, curr->descriptors, matches);
    //     }
    //     else
    //     {
    //         flannMatcher_->match(prev->descriptors, curr->descriptors, matches);
    //     }
    //
    //     sortMatches(matches);
    //     matches.resize(config_.super_point_max_sorted_simple_features);
    //
    //     filterSimpleMatchesAndFillResults(config_.super_point_max_dist_simple, prev, curr, matches, resMatches, resPrevPts, resCurrPts);
    // }
    //
    // return resCurrPts.size() >= config_.super_point_min_valid_features;
}
