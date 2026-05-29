#pragma once

#include "frame.hpp"

class IVisualizer
{
public:
    virtual ~IVisualizer() = default;

    virtual void drawMatches(
        Frame::Ptr prev,
        Frame::Ptr curr,
        const std::vector<cv::DMatch>& matches
    ) = 0;

    virtual void drawPose(
        const Sophus::SE3d& pose,
        const Sophus::SE3d& gt_pose
    ) = 0;
};
