#include "camera_intrinsics.hpp"
#include <utils.hpp>
#include <iostream>
#include <stdexcept>
#include <format>

cv::Mat CameraIntrinsics::K() const
{
    return (cv::Mat_<double>(3,3) <<
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);
}

cv::FileStorage CameraIntrinsics::_readYaml(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        throw std::invalid_argument(
            std::format("Failed to open camera config: {}", path)
        );
    }

    return fs;
}

bool CameraIntrinsics::loadFromYaml(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cerr
            << "Failed to open camera config: "
            << path
            << std::endl;

        return false;
    }

    cv::FileNode camera = fs["camera"];

    fx = round_with_precision((double)camera["fx"], precision);
    fy = round_with_precision((double)camera["fy"], precision);
    cx = round_with_precision((double)camera["cx"], precision);
    cy = round_with_precision((double)camera["cy"], precision);

    return true;
}

void CameraIntrinsics::dump() const
{
    std::cout
        << "Loaded camera intrinsics:" << std::endl
        << "fx=" << fx << std::endl
        << "fy=" << fy << std::endl
        << "cx=" << cx << std::endl
        << "cy=" << cy << std::endl;
}
