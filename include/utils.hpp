#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <format>

double roundWithPrecision(double value, int precision);

std::vector<std::string> loadImagesFromPath(const std::string& folder);

cv::FileStorage readYaml(const std::string& path);
