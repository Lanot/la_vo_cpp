#include "config.hpp"
#include <utils.hpp>
#include <iostream>
#include <format>

Config::Config(const std::string& path)
{
    fs_ = readYaml(path);

    loadIntrinsics();
    loadOptimization();
    loadPoseEstimator();
    loadTrackingFeatureOrb();
}

bool Config::validate() const
{
    if (!(cam_.fx > 0 && cam_.fy > 0 && cam_.cx > 0 && cam_.cy > 0))
    {
        throw std::invalid_argument("Invalid camera intrinsics configuration.");
    }

    // @todo: validate estimation
    // @todo: validate orb

    return true;
}

cv::Mat Config::K() const
{
    return (cv::Mat_<double>(3,3) <<
        cam_.fx, 0, cam_.cx,
        0, cam_.fy, cam_.cy,
        0, 0, 1);
}


OptimizationConfig Config::optimizationConfig() const
{
    return optimization_;
}

PoseEstimatorConfig Config::poseEstimatorConfig() const
{
    return pose_estimator_config_;
}

void Config::loadIntrinsics()
{
    const cv::FileNode intrinsics = fs_["camera"]["intrinsics"];
    //
    cam_.fx = roundWithPrecision((double)intrinsics["fx"], precision_);
    cam_.fy = roundWithPrecision((double)intrinsics["fy"], precision_);
    cam_.cx = roundWithPrecision((double)intrinsics["cx"], precision_);
    cam_.cy = roundWithPrecision((double)intrinsics["cy"], precision_);
}

void Config::loadOptimization()
{
    const cv::FileNode opt = fs_["optimization"];
    //
    opt["use_clahe"] >> optimization_.use_clahe;
}

void Config::loadPoseEstimator()
{
    const cv::FileNode est = fs_["estimation"];
    //
    est["min_pts"] >> pose_estimator_config_.min_pts;
    est["essential_prob"] >> pose_estimator_config_.essential_prob;
    est["essential_threshold"] >> pose_estimator_config_.essential_threshold;
}

void Config::loadTrackingFeatureOrb()
{
    const cv::FileNode tracking = fs_["tracking"];
    //

}
