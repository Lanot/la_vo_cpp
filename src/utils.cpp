#include "utils.hpp"

#include <filesystem>
#include <algorithm>
#include <cmath>

std::vector<std::string> loadImages(const std::string& folder)
{
    std::vector<std::string> files;

    for (auto& entry : std::filesystem::directory_iterator(folder))
    {
        files.push_back(entry.path().string());
    }

    std::sort(files.begin(), files.end());

    return files;
}

double round_with_precision(double value, int precision)
{
    double power_of_10 = std::pow(10.0, precision);
    return std::round(value * power_of_10) / power_of_10;
}
