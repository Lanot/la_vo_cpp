#pragma once

#include "ivisualizer.hpp"

class OpenCvVisualizer : public IVisualizer
{
public:
    void drawMatches(
        Frame::Ptr prev,
        Frame::Ptr curr,
        const std::vector<cv::DMatch>& matches
    ) override;

    void drawPose(
        const Sophus::SE3d& pose
    ) override;
};
