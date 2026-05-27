#pragma once

#include <opencv2/opencv.hpp>

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
    double essential_prob = 0.999;
    double essential_threshold = 1.0;
};

struct TrackingFeatureOrbConfig
{

};

class Config
{
protected:
    void loadIntrinsics();
    void loadOptimization();
    void loadPoseEstimator();
    void loadTrackingFeatureOrb();

    int precision_ = 4;

    cv::FileStorage fs_;
    CameraConfig cam_;
    OptimizationConfig optimization_;

    TrackingFeatureOrbConfig tracking_feature_orb_;
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
};
