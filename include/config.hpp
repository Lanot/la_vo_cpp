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

class Config
{
protected:
    void loadIntrinsics();
    void loadOptimization();

    int precision_ = 4;

    cv::FileStorage fs_;
    CameraConfig cam_;
    OptimizationConfig optimization_;

public:
    Config(const std::string& path);
    bool validate() const;

    // intrinsic matrix
    cv::Mat K() const;

    // Return Optimization Config
    [[nodiscard]] OptimizationConfig O() const;
};
