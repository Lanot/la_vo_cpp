#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <sophus/se3.hpp>

double roundWithPrecision(double value, int precision);

std::vector<std::string> loadImagesFromPath(const std::string& folder);

std::vector<std::string> loadPosesFromPath(const std::string& file);

cv::FileStorage readYaml(const std::string& path);

Sophus::SE3d kittiLinePoseToSophusPose(std::string line);

void sortDMatches(std::vector<cv::DMatch>& matches);