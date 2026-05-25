#pragma once

#include <opencv2/opencv.hpp>

class CameraIntrinsics
{
public:
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;

    int precision = 4;

    cv::Mat K() const;

    bool loadFromYaml(const std::string& path);

    cv::FileStorage _readYaml(const std::string& path);

    void dump() const;
};
