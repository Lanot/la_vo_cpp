#include "utils.hpp"

#include <filesystem>
#include <algorithm>
#include <cmath>

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
