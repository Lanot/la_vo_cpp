#include "utils.hpp"

#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cmath>

#include <sophus/se3.hpp>
#include <Eigen/Core>

double roundWithPrecision(double value, int precision)
{
    double power_of_10 = std::pow(10.0, precision);
    return std::round(value * power_of_10) / power_of_10;
}

std::vector<std::string> loadImagesFromPath(const std::string& folder)
{
    std::vector<std::string> files;

    for (auto& entry : std::filesystem::directory_iterator(folder))
    {
        files.push_back(entry.path().string());
    }

    std::sort(files.begin(), files.end());

    return files;
}

std::vector<std::string> loadPosesFromPath(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
    {
        throw std::runtime_error(
            "Cannot open file: " + path
        );
    }

    std::vector<std::string> lines;

    for (std::string line; std::getline(file, line);)
    {
        lines.push_back(line);
    }

    return lines;
}

cv::FileStorage readYaml(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        throw std::invalid_argument(
            std::format("Failed to open file: {}", path)
        );
    }

    return fs;
}

Sophus::SE3d kittiLinePoseToSophusPose(std::string line)
{
    std::stringstream ss(line);

    Eigen::Matrix3d R =
        Eigen::Matrix3d::Identity();

    Eigen::Vector3d t =
        Eigen::Vector3d::Zero();

    if (!(ss
        >> R(0,0) >> R(0,1) >> R(0,2) >> t(0)
        >> R(1,0) >> R(1,1) >> R(1,2) >> t(1)
        >> R(2,0) >> R(2,1) >> R(2,2) >> t(2)))
    {
        throw std::runtime_error(
            "Invalid KITTI pose line"
        );
    }

    // Normalize Rotation upon using Quaternion, in other case it fails to create Sophus:SE3d()
    Eigen::Quaterniond q(R);
    q.normalize();
    R = q.toRotationMatrix();

    return Sophus::SE3d(R, t);
}

void sortMatches(std::vector<cv::DMatch>& matches)
{
    std::sort(
        matches.begin(),
        matches.end(),
        [](const cv::DMatch& a, const cv::DMatch& b)
        {
            return a.distance < b.distance;
        }
    );
}

void estimateOpticalFlowPyrLKMatchesAndFillResults(
    cv::TermCriteria& termCriteria,
    const Frame::Ptr& prev,
    const Frame::Ptr& curr,
    std::vector<cv::Point2f>& resPrevPts,
    std::vector<cv::Point2f>& resCurrPts
)
{
    std::vector<uchar> status;
    std::vector<float> error;

    std::vector<cv::Point2f> prev_points2f;
    std::vector<cv::Point2f> curr_points2f;

    // Optical Flow Support
    cv::KeyPoint::convert(prev->keypoints, prev_points2f);
    cv::KeyPoint::convert(curr->keypoints, curr_points2f);

    cv::calcOpticalFlowPyrLK(
    prev->image,
        curr->image,
        prev_points2f,
        curr_points2f,
        status,
        error,
        cv::Size(21, 21),
        3,
        termCriteria
    );

    for (size_t i = 0; i < status.size(); ++i)
    {
        if (status[i])
        {
            resPrevPts.push_back(prev_points2f[i]);
            resCurrPts.push_back(curr_points2f[i]);
        }
    }
}

//queryIdx	index in FIRST descriptor set
//trainIdx	index in SECOND descriptor set
void filterSimpleMatchesAndFillResults(
    double max_dist,
    const Frame::Ptr& prev,
    const Frame::Ptr& curr,
    const std::vector<cv::DMatch>& simpleMatches,
    std::vector<cv::DMatch>& resMatches,
    std::vector<cv::Point2f>& resPrevPts,
    std::vector<cv::Point2f>& resCurrPts
)
{
    //todo: resize: matches.resize(config_.orb_max_sorted_features);
    for (auto& m : simpleMatches)
    {
        if (m.distance <= max_dist)
        {
            resMatches.push_back(m);
            resPrevPts.push_back(prev->keypoints[m.queryIdx].pt);
            resCurrPts.push_back(curr->keypoints[m.trainIdx].pt);
        }
    }
}

void filterKnnMatchesAndFillResults(
    double knn_dist_k,
    const Frame::Ptr& prev,
    const Frame::Ptr& curr,
    std::vector<std::vector<cv::DMatch>>& knnMatches,
    std::vector<cv::DMatch>& resMatches,
    std::vector<cv::Point2f>& resPrevPts,
    std::vector<cv::Point2f>& resCurrPts
)
{
    for (std::vector<cv::DMatch>& m : knnMatches)
    {
        if (m.size() < 2)
            continue;

        if (m[0].distance < knn_dist_k * m[1].distance) // && (m[0].distance < config_.orb_max_dist))
        {
            resMatches.push_back(m[0]);
            resPrevPts.push_back(prev->keypoints[m[0].queryIdx].pt);
            resCurrPts.push_back(curr->keypoints[m[0].trainIdx].pt);
        }
    }
}
