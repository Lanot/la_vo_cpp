#pragma once

#include <opencv2/opencv.hpp>

struct CameraConfig
{
    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;
};

class Config
{
protected:
    bool loadFromYaml(const std::string& path);

    CameraConfig cam_;
    int precision = 4;

public:
    Config(const std::string& path);

    cv::Mat K() const;

    bool validate() const;
    void dump() const;
};
