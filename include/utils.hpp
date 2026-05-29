#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <sophus/se3.hpp>

#include "frame.hpp"

double roundWithPrecision(double value, int precision);

std::vector<std::string> loadImagesFromPath(const std::string& folder);

std::vector<std::string> loadPosesFromPath(const std::string& file);

cv::FileStorage readYaml(const std::string& path);

Sophus::SE3d kittiLinePoseToSophusPose(std::string line);

void sortMatches(std::vector<cv::DMatch>& matches);

void estimateOpticalFlowPyrLKMatchesAndFillResults(
    cv::TermCriteria& termCriteria,
    const Frame::Ptr& prev,
    const Frame::Ptr& curr,
    // std::vector<cv::DMatch>& resMatches,
    std::vector<cv::Point2f>& resPrevPts,
    std::vector<cv::Point2f>& resCurrPts
);

void filterSimpleMatchesAndFillResults(
    double max_dist,
    const Frame::Ptr& prev,
    const Frame::Ptr& curr,
    const std::vector<cv::DMatch>& matches,
    std::vector<cv::DMatch>& resMatches,
    std::vector<cv::Point2f>& resPrevPts,
    std::vector<cv::Point2f>& resCurrPts
);

void filterKnnMatchesAndFillResults(
    double knn_dist_k,
    const Frame::Ptr& prev,
    const Frame::Ptr& curr,
    std::vector<std::vector<cv::DMatch>>& knnMatches,
    std::vector<cv::DMatch>& resMatches,
    std::vector<cv::Point2f>& resPrevPts,
    std::vector<cv::Point2f>& resCurrPts
);
