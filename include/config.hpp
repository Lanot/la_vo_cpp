#pragma once

#include <opencv2/opencv.hpp>

enum FeatureMatcher
{
    BF,
    BF_KNN,
    FLANN,
    FLANN_KNN,
};

enum FeatureType
{
    ORB,
    SIFT,
    SUPER_POINT
};

struct CameraConfig
{
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
};

struct OptimizationConfig
{
    bool use_clahe = false;
};

struct PoseEstimatorConfig
{
    int min_pts = 8;
    int min_inliers = 15;

    double essential_prob = 0.999;
    double essential_threshold = 1.0;
};

struct TrackerConfig
{
    FeatureType feature_type = FeatureType::ORB;

    FeatureMatcher orb_feature_matcher = FeatureMatcher::BF;
    int orb_max_extract_features = 1000;
    int orb_max_sorted_simple_features = 100;
    int orb_min_valid_features = 20;
    double orb_knn_dist_k = 0.8;
    double orb_max_dist_simple = 20;

    FeatureMatcher sift_feature_matcher = FeatureMatcher::BF;
    int sift_max_extract_features = 1000;
    int sift_max_sorted_simple_features = 100;
    int sift_min_valid_features = 20;
    double sift_knn_dist_k = 0.8;
    double sift_max_dist_simple = 100;
};

// ------------------------------------------------------------------------------------------------------------------ //
class Config
{
protected:
    void loadIntrinsics();
    void loadOptimization();
    void loadPoseEstimator();
    void loadTracking();

    static FeatureType mapFeatureType(const std::string& feature_type);
    static FeatureMatcher mapFeatureMatcher(const std::string& feature_matcher);

    int precision_ = 4;

    cv::FileStorage fs_;

    CameraConfig cam_;
    OptimizationConfig optimization_;

    TrackerConfig tracker_;
    PoseEstimatorConfig pose_estimator_config_;


public:
    Config(const std::string& path);
    bool validate() const;

    // intrinsic matrix
    cv::Mat K() const;

    // Return Optimization Config
    [[nodiscard]] OptimizationConfig optimizationConfig() const;

    // Return Optimization Config
    [[nodiscard]] PoseEstimatorConfig poseEstimatorConfig() const;

    // Return Optimization Config
    [[nodiscard]] TrackerConfig trackerConfig() const;
};
