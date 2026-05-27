#include "config.hpp"
#include <utils.hpp>
#include <iostream>
#include <format>

Config::Config(const std::string& path)
{
    loadFromYaml(path);
}

cv::Mat Config::K() const
{
    return (cv::Mat_<double>(3,3) <<
        cam_.fx, 0, cam_.cx,
        0, cam_.fy, cam_.cy,
        0, 0, 1);
}

bool Config::loadFromYaml(const std::string& path)
{
    cv::FileStorage fs = readYaml(path);

    cv::FileNode intrinsics = fs["camera"]["intrinsics"];
    //
    cam_.fx = roundWithPrecision((double)intrinsics["fx"], precision);
    cam_.fy = roundWithPrecision((double)intrinsics["fy"], precision);
    cam_.cx = roundWithPrecision((double)intrinsics["cx"], precision);
    cam_.cy = roundWithPrecision((double)intrinsics["cy"], precision);

    return true;
}

void Config::dump() const
{
    std::cout
        << "Loaded camera intrinsics:" << std::endl
        << "fx=" << cam_.fx << std::endl
        << "fy=" << cam_.fy << std::endl
        << "cx=" << cam_.cx << std::endl
        << "cy=" << cam_.cy << std::endl;
}

bool Config::validate() const
{
    if (!(cam_.fx > 0 && cam_.fy > 0 && cam_.cx > 0 && cam_.cy > 0))
    {
        throw std::invalid_argument("Invalid camera intrinsics configuration.");
    }

    return true;
}